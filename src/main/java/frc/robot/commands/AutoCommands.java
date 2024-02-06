package frc.robot.commands;

import java.util.HashMap;

import javax.swing.GroupLayout.SequentialGroup;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
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

  public Command folllowTrajectory(String PathName){
    PathPlannerPath path = PathPlannerPath.fromPathFile(PathName);
    return AutoBuilder.followPath(path);
  }
    
  public Command score() {
    return new RunCommand(() -> {
      indexer.setIndexerSpeed(0.2);
      shooter.setShootingSpeed(0.5);
    }).until(() -> pivot.atSetpoint());
}

  public Command Intake() {
    return new RunCommand(() -> {
    intake.setIntakeSpeed(0.5);
  });
}

  public Command SetPivotAngle(double setpoint){
    return new RunCommand(() -> {
      pivot.setGoal(setpoint);
    });
  }

}