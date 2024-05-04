package frc.robot.autoPaths;

import java.util.List;

import org.photonvision.PhotonTargetSortMode;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PivotSetpoints;
import frc.robot.commands.AutoCommands;
import frc.robot.subsystems.vision.intakeCam.IntakeCamVision;

public class GDA_SS8765 {
    private static List<PathPlannerPath> paths;
    private static PathPlannerPath mainPath = PathPlannerPath.fromPathFile("SS GP 8765 P7");
    private static PathPlannerPath altPath = PathPlannerPath.fromPathFile("SS GP 8765 P7 ALT");
    private static PathPlannerPath scorePath = PathPlannerPath.fromPathFile("SS GP 8765 P8");
    PathPlannerAuto auto;

    public static Command load(){
        paths = PathPlannerAuto.getPathGroupFromAutoFile("SS GP 876");
        IntakeCamVision.getInstance().setTargetComparator(PhotonTargetSortMode.Centermost);

        return Commands.sequence(
            Commands.parallel(
                AutoCommands.rotateToSpeaker(),
                AutoCommands.aimSpeakerDynamic(true, 5000)
            ),
            AutoCommands.score(),
            
            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(0)),
                AutoCommands.followTrajectory(paths.get(1)),
                Commands.parallel(
                    AutoCommands.aimSpeakerDynamic(true, 5000),
                    AutoCommands.rotateToSpeaker()
                ),
                Commands.waitSeconds(0.2),
                AutoCommands.score()
            ).until(() -> AutoGamepieces.isGone(8)),

            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(2)),
                AutoCommands.followTrajectory(paths.get(3)),
                Commands.parallel(
                    AutoCommands.aimSpeakerDynamic(true, 5000),
                    AutoCommands.rotateToSpeaker()
                ),
                Commands.waitSeconds(0.2),
                AutoCommands.score()
            ).until(() -> AutoGamepieces.isGone(7)),

            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(4)),
                AutoCommands.SetPivotAngle(PivotSetpoints.stow),
                AutoCommands.followTrajectory(paths.get(5)),
                Commands.parallel(
                    AutoCommands.aimSpeakerDynamic(true, 5000),
                    AutoCommands.rotateToSpeaker()
                ),
                Commands.waitSeconds(0.2),
                AutoCommands.score()
            ).until(() -> AutoGamepieces.isGone(6)),

            Commands.sequence(
                Commands.either(
                    AutoCommands.intakeTrajectory(mainPath), 
                    AutoCommands.intakeTrajectory(altPath), 
                    () -> AutoGamepieces.isGone(6)
                ),
                AutoCommands.followTrajectory(scorePath),
                Commands.parallel(
                    AutoCommands.aimSpeakerDynamic(true, 5000),
                    AutoCommands.rotateToSpeaker()
                ),
                Commands.waitSeconds(0.2),
                AutoCommands.score()
            )
        );
    }
}
