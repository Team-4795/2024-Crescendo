package frc.robot.autoPaths;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoCommands;

public abstract class TestingAuto extends AutoPath{
    public Command load(AutoCommands autoCommands) {
        return Commands.sequence(
        AutoCommands.initialize(1),
        Commands.waitSeconds(2),
        AutoCommands.score(),
        AutoCommands.alignTrajectory("Testing Auto", 0),
        AutoCommands.alignTrajectory("Testing Auto P2", 0)
        );
    }
}