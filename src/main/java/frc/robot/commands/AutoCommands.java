package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.StateManager;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;

public class AutoCommands {
  private HashMap<String, PathPlannerTrajectory> paths = new HashMap<>();

  private static Drive drive = Drive.getInstance();
  private static Shooter shooter = Shooter.getInstance();
  private static Intake intake = Intake.getInstance();
  private static Pivot pivot = Pivot.getInstance();
  private static Indexer indexer = Indexer.getInstance();
  private StateManager manager = StateManager.getInstance();

  public enum OutakePlace {
    amp,
    speaker;
  }

  public AutoCommands() {
  }

  public static Command followTrajectory(String PathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(PathName);
    return AutoBuilder.followPath(path);
  }

  public static Command score() {
    return indexer.forwards().withTimeout(0.5);
  }

  public static Command SetPivotAngle(double setpoint) {
    return new InstantCommand(() -> {
      pivot.setGoal(setpoint);
    });
  }

  public static Command alignTrajectory(String PathName, double setpoint) {
    return Commands.parallel(
        followTrajectory(PathName),
        SetPivotAngle(setpoint));
  };

  public static Command initialize(double speed) {
    return new InstantCommand(() -> {
      intake.setIntakeSpeed(speed);
      shooter.setShootingSpeedRPM(-3000 * speed, 3000 * speed);
    });
  }

  public static Command resetOdometry(Pose2d pose) {
    return new InstantCommand(() -> {
      drive.resetOdometry(pose);
    });
  }

  public static Command runIndexer(double speed) {
    return new InstantCommand(() -> {
      indexer.setIndexerSpeed(0.7);
    });
  }

  public static Command runEverything(double speed) {
    return Commands.parallel(
        runIndexer(speed),
        initialize(speed));
  }

  public static Command intake() {
    return Commands.parallel(
      intake.intake().until(indexer::handoff),
      indexer.forwards()
    ).until(indexer::isStoring)
     .andThen(indexer.reverse().withTimeout(0.1)
    );
  }

  public static Command sensingPiece() {
    return Commands.waitUntil(() -> indexer.isStoring());
  }
}
