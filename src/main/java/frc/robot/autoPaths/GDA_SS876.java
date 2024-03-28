package frc.robot.autoPaths;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoCommands;

public class GDA_SS876 {
    private static List<PathPlannerPath> paths;
    PathPlannerAuto auto;

    public static Command load(){
        paths = PathPlannerAuto.getPathGroupFromAutoFile("SS GP 876");
        
        return Commands.sequence(
            Commands.parallel(
                AutoCommands.rotateToSpeaker(),
                AutoCommands.aimSpeakerDynamic(true)
            ),
            AutoCommands.score(),
            
            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(0)),
                Commands.deadline(
                    Commands.sequence(
                        AutoCommands.followTrajectory(paths.get(1)), //contains a instant command to start vision align near the end
                        Commands.waitSeconds(0.2)
                    ), AutoCommands.aimSpeakerDynamic(false)
                ),
                AutoCommands.score()
            ).until(() -> AutoGamepieces.isGone(8)),

            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(2)),
                Commands.deadline(
                    Commands.sequence(
                        AutoCommands.followTrajectory(paths.get(3)), //contains a instant command to start vision align near the end
                        Commands.waitSeconds(0.2)
                    ), AutoCommands.aimSpeakerDynamic(false)
                ),
                AutoCommands.score()
            ).until(() -> AutoGamepieces.isGone(7)),

            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(4)),
                Commands.deadline(
                    Commands.sequence(
                        AutoCommands.followTrajectory(paths.get(5)), //contains a instant command to start vision align near the end
                        Commands.waitSeconds(0.2)
                    ), AutoCommands.aimSpeakerDynamic(false)
                ),
                AutoCommands.score()
            )
        );
    }
}
