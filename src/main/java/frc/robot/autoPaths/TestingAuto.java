package frc.robot.autoPaths;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoCommands;

public abstract class TestingAuto extends AutoPath{
    public Command load(AutoCommands autoCommands) {
        return Commands.sequence(
        autoCommands.initialize(1),
        Commands.waitSeconds(2),
        autoCommands.score(0.5),
        autoCommands.alignTrajectory("Testing Auto", 0),
        autoCommands.alignTrajectory("Testing Auto P2", 0)
        );
    }
}