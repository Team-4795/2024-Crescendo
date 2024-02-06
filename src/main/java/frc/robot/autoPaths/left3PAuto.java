package frc.robot.autoPaths;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoCommands;

public abstract class left3PAuto extends AutoPath{
    public Command load(AutoCommands autoCommands) {
        return Commands.sequence(
        autoCommands.score(),
        autoCommands.followTrajectory("Left GP3 P1"),
        autoCommands.intake(),
        autoCommands.score(),
        autoCommands.followTrajectory("Left GP3 P2"),
        autoCommands.intake()
        );
    }
}