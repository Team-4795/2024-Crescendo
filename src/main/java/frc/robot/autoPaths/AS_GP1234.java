package frc.robot.autoPaths;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoCommands;

public class 
AS_GP1234 extends AutoPath{
    public Command load(AutoCommands autoCommands) {
        return Commands.sequence(
            AutoCommands.initialize(1),
            AutoCommands.score(),
            AutoCommands.alignTrajectory("AS GP1234 P1", AutoConstants.closePivotSetpoint),
            //change pivot angles later
            AutoCommands.score(),
            AutoCommands.alignTrajectory("AS GP1234 P2", AutoConstants.closePivotSetpoint),
            AutoCommands.score(),
            AutoCommands.alignTrajectory("AS GP1234 P3", AutoConstants.closePivotSetpoint),
            AutoCommands.score(),
            AutoCommands.alignTrajectory("AS GP1234 P4", 0), //change later
            AutoCommands.score()
        );
    }
}

