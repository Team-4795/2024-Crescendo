package frc.robot.autoPaths;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoCommands;

public class AS_GP123 extends AutoPath{
    public Command load(AutoCommands autoCommands) {
        return Commands.sequence(
            autoCommands.initialize(1),
            autoCommands.score(0.5),
            autoCommands.alignTrajectory("AS GP123 P1", AutoConstants.closePivotSetpoint),
            autoCommands.score(0.5),
            autoCommands.alignTrajectory("AS GP123 P2", AutoConstants.closePivotSetpoint),
            autoCommands.score(0.5),
            autoCommands.alignTrajectory("AS GP123 P3", AutoConstants.closePivotSetpoint),
            autoCommands.score(0.5)
        );
    }
}

