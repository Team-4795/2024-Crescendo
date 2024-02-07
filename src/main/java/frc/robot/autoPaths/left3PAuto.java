package frc.robot.autoPaths;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoCommands;

public abstract class left3PAuto extends AutoPath{
    public Command load(AutoCommands autoCommands) {
        return Commands.sequence(
        autoCommands.initialize(1),
        autoCommands.score(),
        autoCommands.followTrajectory("Left GP3 P1"),
        autoCommands.score(),
        autoCommands.followTrajectory("Left GP3 P2")
        );
    }
}