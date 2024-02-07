package frc.robot.autoPaths;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands;


public class Close3PAuto extends AutoPath {
    public Command load(AutoCommands autoCommands) {
        return Commands.sequence(
            autoCommands.initialize(1),
            autoCommands.score(),
            autoCommands.followTrajectory("3GP middle P1"),
            autoCommands.score(),
            autoCommands.followTrajectory("3GP middle P2"),
            autoCommands.SetPivotAngle(0.2),//insert proper angle here later
            autoCommands.score()
        );
    }
}