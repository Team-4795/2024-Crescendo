package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IndexerSetpoints;
import frc.robot.Constants.ShooterSetpoints;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.NoteVisualizer;

public class AutoCommands {
  private HashMap<String, PathPlannerTrajectory> paths = new HashMap<>();

  private static Drive drive = Drive.getInstance();
  private static Shooter shooter = Shooter.getInstance();
  private static Intake intake = Intake.getInstance();
  private static Pivot pivot = Pivot.getInstance();
  private static Indexer indexer = Indexer.getInstance();

  public AutoCommands() {
  }

  public static Command followTrajectory(String PathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(PathName);
    return AutoBuilder.followPath(path);
  }

  public static Command score() {
    return Commands.sequence(
          indexer.forwards().withTimeout(0.4).alongWith(Commands.waitSeconds(0.1).andThen(NoteVisualizer.shoot())),
          Commands.runOnce(() -> shooter.setShootingSpeedRPM(0.0, 0.0))
    );
  }

  public static Command SetPivotAngle(double setpoint) {
    return new InstantCommand(() -> {
      pivot.setGoal(setpoint);
    });
  }

  public static Command setPivotAndShooter(double setpoint){
    return Commands.parallel(
      Commands.runOnce(() -> pivot.setGoal(setpoint)),
      Commands.runOnce(() -> shooter.setShootingSpeedRPM(ShooterSetpoints.speakerTop, ShooterSetpoints.speakerBottom))
    );
  }

  public static Command alignTrajectory(String PathName, double setpoint) {
    return Commands.parallel(
        followTrajectory(PathName),
        SetPivotAngle(setpoint));
  };

  public static Command initialize(double speed) {
    return Commands.parallel(
      Commands.runOnce(() -> intake.setIntakeSpeed(-1)),
      Commands.runOnce(() -> shooter.setShootingSpeedRPM(ShooterSetpoints.speakerTop, ShooterSetpoints.speakerBottom))
    );
  }

  public static Command resetOdometry(Pose2d pose) {
    return new InstantCommand(() -> {
      drive.resetOdometry(pose);
    });
  }

  public static Command runIndexer(double speed) {
    return new InstantCommand(() -> {
      indexer.setIndexerSpeed(1);
    });
  }

  public static Command runEverything(double speed) {
    return Commands.parallel(
        runIndexer(speed),
        initialize(speed));
  }

  public static Command intake() {
    return Commands.sequence(
      Commands.parallel(
        Commands.runOnce(() -> indexer.setIndexerSpeed(IndexerSetpoints.shoot)),
        Commands.runOnce(() -> pivot.setGoal(0.55))
      ),
      Commands.either(Commands.waitUntil(indexer::isStoring), Commands.waitSeconds(1), Robot::isReal),
      indexer.reverse().withTimeout(0.1));
  }

  public static Command sensingPiece() {
    return Commands.waitUntil(() -> indexer.isStoring());
  }

  public static Command aimSpeakerDynamic(){
    return Commands.run(() -> {
            double distanceToSpeaker = Vision.getInstance().getDistancetoSpeaker(Drive.getInstance().getPose());
            double angleCalc = Math.atan((FieldConstants.speakerHeight - PivotConstants.height) / (distanceToSpeaker + PivotConstants.offset));
            pivot.setGoal(angleCalc - PivotConstants.angleOffset);
        });
  }

}
