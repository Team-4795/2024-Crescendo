package frc.robot.autoPaths;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoCommands;

public class 
AS_GP1234 extends AutoPath{
    public Command load(AutoCommands autoCommands) {
        return Commands.sequence(
            autoCommands.initialize(1),
            autoCommands.score(1),
            autoCommands.alignTrajectory("AS GP1234 P1", AutoConstants.closePivotSetpoint),
            //change pivot angles later
            autoCommands.score(1),
            autoCommands.alignTrajectory("AS GP1234 P2", AutoConstants.closePivotSetpoint),
            autoCommands.score(1),
            autoCommands.alignTrajectory("AS GP1234 P3", AutoConstants.closePivotSetpoint),
            autoCommands.score(1),
            autoCommands.alignTrajectory("AS GP1234 P4", 0), //change later
            autoCommands.score(1)
        );
    }
}

