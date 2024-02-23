package frc.robot.autoPaths;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands;
    
// drive sim is somewhat off, will look into that
//uncommented because, pivot will never reach, so score will run forever, also using runcommands gotta change that

public class testingPath extends AutoPath {
    public Command load(AutoCommands autoCommands) {
        return Commands.sequence(
            autoCommands.initialize(1),
            autoCommands.alignTrajectory("Go forward", 0)
        );
    }
}