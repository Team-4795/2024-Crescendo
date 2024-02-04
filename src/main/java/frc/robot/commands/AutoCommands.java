package frc.robot.commands;

import java.util.HashMap;

import javax.swing.GroupLayout.SequentialGroup;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.StateManager;
import frc.robot.StateManager.State;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.MAXSwerve.DriveConstants.AutoConstants;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;

public class AutoCommands {
    private HashMap<String, PathPlannerTrajectory> paths = new HashMap<>();

    private Drive drive = Drive.getInstance();
    private Shooter shooter = Shooter.getInstance();
    private Intake intake = Intake.getInstance();
    private Pivot pivot = Pivot.getInstance();
    private Indexer indexer = Indexer.getInstance();
    private StateManager manager = StateManager.getInstance();

    public enum OutakePlace{
      amp, 
      speaker;
    }

  public AutoCommands() {}
    
  public Command score() {
    return new RunCommand(() -> {
      indexer.setIndexerSpeed(0.2);
      shooter.setShootingSpeed(0.5);
    });
}

  public Command Intake() {
    return new RunCommand(() -> {
    intake.setIntakeSpeed(0.5);
  });
}


    //   public Command scoreTrajectory(OutakePlace outakePlace, PathPlannerTrajectory traj) {
    //     return Commands.parallel(
    //             drive.followTrajectoryCommand(traj),
    //             Commands.sequence(
    //                     Commands.waitSeconds(AutoConstants.kIntakeDelay), score(gamepiece, height, backwards)));
    // }

    // public Command intakeTrajectory(PathPlannerTrajectory traj) {
    //     return intakeTrajectory(traj, 0.0);

    // public Command stow() {
    //     return new ChangeStateCommand(State.Stow, intake, true, manager);
    //   }
    // }
}