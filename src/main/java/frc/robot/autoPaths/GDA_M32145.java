package frc.robot.autoPaths;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoCommands;

public class GDA_M32145 {
    private static List<PathPlannerPath> paths;
    PathPlannerAuto auto;

    public static Command load(){
        paths = PathPlannerAuto.getPathGroupFromAutoFile("GDA M GP 32145");

        return Commands.sequence(
            AutoCommands.aimSpeakerDynamic(true),
            AutoCommands.score(),

        Commands.sequence(
            AutoCommands.intakeTrajectory(paths.get(0)),
            Commands.waitSeconds(0.2), 
            AutoCommands.aimSpeakerDynamic(true)),

        AutoCommands.score(),
            
        Commands.sequence(
            AutoCommands.intakeTrajectory(paths.get(1)),
            Commands.waitSeconds(0.2), 
            AutoCommands.aimSpeakerDynamic(true)),

            AutoCommands.score(),
            
        Commands.sequence(
            AutoCommands.intakeTrajectory(paths.get(2)),
            Commands.waitSeconds(0.2)),
            AutoCommands.aimSpeakerDynamic(true),

            AutoCommands.score(),
            
            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(3)),
                Commands.deadline( 
                    Commands.sequence(
                        //AutoCommands.SetPivotAngle(.129),
                        AutoCommands.followTrajectory(paths.get(4)), 
                        Commands.waitSeconds(0.2)
                    ), AutoCommands.aimSpeakerDynamic(true)),
                AutoCommands.score()
            ).until(() -> AutoGamepieces.isGone(4)),

            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(5)),
                Commands.deadline(
                    Commands.sequence(
                        AutoCommands.followTrajectory(paths.get(6)), 
                        Commands.waitSeconds(0.2)
                    ), AutoCommands.aimSpeakerDynamic(true)),

                AutoCommands.score()
            ).until(() -> AutoGamepieces.isGone(5)));

    }
}