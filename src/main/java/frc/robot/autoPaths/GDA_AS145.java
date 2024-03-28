package frc.robot.autoPaths;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoCommands;

public class GDA_AS145 {
    private static List<PathPlannerPath> paths;
    PathPlannerAuto auto;

    public static Command load(){
        paths = PathPlannerAuto.getPathGroupFromAutoFile("AS GP 145");
        
        return Commands.sequence(
            Commands.parallel(
                AutoCommands.rotateToSpeaker(),
                AutoCommands.aimSpeakerDynamic(true)
            ),
            AutoCommands.score(),
            
            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(0)),
                Commands.waitSeconds(0.2),
                AutoCommands.aimSpeakerDynamic(true),
                AutoCommands.score()
            ),

            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(1)),
                Commands.deadline(
                    Commands.sequence(
                        AutoCommands.followTrajectory(paths.get(2)), //contains a instant command to start vision align near the end
                        Commands.waitSeconds(0.2)
                    ), AutoCommands.aimSpeakerDynamic(true)
                ),
                AutoCommands.score()
            ).until(() -> AutoGamepieces.isGone(4)),

            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(3)),
                Commands.deadline(
                    Commands.sequence(
                        AutoCommands.followTrajectory(paths.get(4)), //contains a instant command to start vision align near the end
                        Commands.waitSeconds(0.2)
                    ), AutoCommands.aimSpeakerDynamic(true)
                ),
                AutoCommands.score()
            ).until(() -> AutoGamepieces.isGone(5)),

            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(5)),
                Commands.deadline(
                    Commands.sequence(
                        AutoCommands.followTrajectory(paths.get(6)), //contains a instant command to start vision align near the end
                        Commands.waitSeconds(0.2)
                    ), AutoCommands.aimSpeakerDynamic(true)
                ),
                AutoCommands.score()
            )
        );
    }
}
