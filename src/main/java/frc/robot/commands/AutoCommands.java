package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.autoPaths.AutoGamepieces;
import frc.robot.Constants;
import frc.robot.Constants.IndexerSetpoints;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ShooterSetpoints;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.vision.AprilTagVision.Vision;
import frc.robot.util.NoteVisualizer;

public class AutoCommands {

  private static Drive drive = Drive.getInstance();
  private static Shooter shooter = Shooter.getInstance();
  private static Intake intake = Intake.getInstance();
  private static Pivot pivot = Pivot.getInstance();
  private static Indexer indexer = Indexer.getInstance();

  private AutoCommands() {
  }

  public static Command followTrajectory(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    // DriverStation.getAlliance().ifPresent((alliance) -> {
    // if(alliance == Alliance.Red){
    // path.flipPath();
    // }
    // });
    return AutoBuilder.followPath(path);
  }

  public static Command followTrajectory(PathPlannerPath path) {
    return AutoBuilder.followPath(path);
  }

  public static Command intakeTrajectory(PathPlannerPath path) {
    return Commands.race(
        Commands.sequence(
            Commands.runOnce(indexer::resetIntakeStatus),
            followTrajectory(path),
            Commands.waitSeconds(0.3)),
        intake());
  }

  public static Command score() {
    return Commands.sequence(
        indexer.forwards().withTimeout(0.6).alongWith(Commands.waitSeconds(0.1).andThen(NoteVisualizer.shoot()))
    // Commands.runOnce(() -> shooter.setShootingSpeedRPM(0.0, 0.0))
    );
  }

  public static Command SetPivotAngle(double setpoint) {
    return new InstantCommand(() -> {
      pivot.setGoal(setpoint);
    });
  }

  public static Command setPivotAndShooter(double setpoint, double shooterRPM) {
    return Commands.parallel(
        Commands.runOnce(() -> pivot.setGoal(setpoint)),
        Commands.runOnce(() -> shooter.setShootingSpeedRPM(-shooterRPM, shooterRPM)));
  }

  public static Command alignTrajectory(String PathName, double setpoint) {
    return Commands.parallel(
        followTrajectory(PathName),
        SetPivotAngle(setpoint));
  };

  public static Command initialize(double shooterRPM) {
    return Commands.parallel(
        Commands.runOnce(() -> intake.setIntakeSpeed(-0.9)),
        Commands.runOnce(() -> shooter.setShootingSpeedRPM(-shooterRPM, shooterRPM)));
  }

  public static Command resetOdometry(Pose2d pose) {
    return Commands.runOnce(() -> {
      drive.resetOdometry(pose);
    });
  }

  public static Command stopShooting() {
    return Commands.parallel(
        Commands.runOnce(() -> indexer.setIndexerSpeed(0)),
        Commands.runOnce(() -> shooter.setShootingSpeedRPM(0, 0)));
  }

  public static Command runEverything(double shooterRPM) {
    return Commands.parallel(
        Commands.runOnce(() -> indexer.setIndexerSpeed(IndexerSetpoints.shoot)),
        initialize(shooterRPM));
  }

  public static Command intake() {
    return Commands.sequence(
        Commands.parallel(
            indexer.forwards(),
            Commands.runOnce(() -> pivot.setGoal(0.3)),
            Commands.runOnce(() -> intake.setIntakeSpeed(-1))).until(indexer::isStoring));
  }

  public static Command intakeWithoutPivot() {
    return Commands.sequence(
        Commands.parallel(
            indexer.forwards(),
            Commands.runOnce(() -> intake.setIntakeSpeed(-1))).until(indexer::isStoring));
  }

  public static Command sensingPiece() {
    return Commands.waitUntil(() -> indexer.isStoring());
  }

  public static Command aimSpeakerDynamic(boolean timeout, double speed) {
    double timeLimit = (timeout) ? 0.7 : 15;
    return Commands.parallel(
        Commands.runOnce(() -> shooter.setShootingSpeedRPM(-speed, speed)),
        Commands.run(() -> {
          double distanceToSpeaker = Vision.getInstance().getDistancetoSpeaker(Drive.getInstance().getPose());
          if (distanceToSpeaker < 8.5) {
            pivot.setGoal(PivotConstants.armAngleMap.get(distanceToSpeaker));
          }
        }).withTimeout(timeLimit));
  }

  public static Command rotateToSpeaker() {
    return new AlignSpeaker().withTimeout(0.7);
  }

  public static Command checkNote(int note, boolean simDetect) {
    return Commands.either(
        Commands.runOnce(() -> AutoGamepieces.setNoteGone(note)).onlyIf(() -> !simDetect),
        Commands.runOnce(() -> AutoGamepieces.setNoteGone(note)).onlyIf(() -> !(indexer.isStoring() || intake.isIntaking())),
        () -> Constants.currentMode == Mode.SIM);
  }

}
