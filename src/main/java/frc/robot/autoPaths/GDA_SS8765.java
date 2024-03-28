package frc.robot.autoPaths;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PivotSetpoints;
import frc.robot.commands.AutoCommands;

public class GDA_SS8765 {
    private static List<PathPlannerPath> paths;
    private static PathPlannerPath mainPath = PathPlannerPath.fromPathFile("SS GP 8765 P7");
    private static PathPlannerPath altPath = PathPlannerPath.fromPathFile("SS GP 8765 P7 ALT");
    private static PathPlannerPath scorePath = PathPlannerPath.fromPathFile("SS GP 8765 P8");
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
                        AutoCommands.followTrajectory(paths.get(1)),
                        Commands.waitSeconds(0.2)
                    ), AutoCommands.aimSpeakerDynamic(false)
                ),
                AutoCommands.score()
            ).until(() -> AutoGamepieces.isGone(8)),

            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(2)),
                Commands.deadline(
                    Commands.sequence(
                        AutoCommands.followTrajectory(paths.get(3)),
                        Commands.waitSeconds(0.2)
                    ), AutoCommands.aimSpeakerDynamic(false)
                ),
                AutoCommands.score()
            ).until(() -> AutoGamepieces.isGone(7)),

            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(4)),
                AutoCommands.SetPivotAngle(PivotSetpoints.stow),
                AutoCommands.followTrajectory(paths.get(5)),
                AutoCommands.aimSpeakerDynamic(true),
                AutoCommands.score()
            ).until(() -> AutoGamepieces.isGone(6)),

            Commands.sequence(
                Commands.either(
                    AutoCommands.intakeTrajectory(mainPath), 
                    AutoCommands.intakeTrajectory(altPath), 
                    () -> AutoGamepieces.isGone(6)
                ),
                Commands.deadline(
                    Commands.sequence(
                        AutoCommands.followTrajectory(scorePath),
                        Commands.waitSeconds(0.2)
                    ), AutoCommands.aimSpeakerDynamic(false)
                ),
                AutoCommands.score()
            )
        );
    }
}
