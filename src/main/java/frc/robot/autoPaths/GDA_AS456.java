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

public class GDA_AS456 {
    private static List<PathPlannerPath> paths;
    PathPlannerAuto auto;

    public static Command load(){
        paths = PathPlannerAuto.getPathGroupFromAutoFile("Choreo AS GP 456");
        IntakeCamVision.getInstance().setTargetComparator(PhotonTargetSortMode.Centermost);
        
        return Commands.sequence(
            AutoCommands.followTrajectory(paths.get(0)),

            Commands.parallel(
                AutoCommands.rotateToSpeaker(),
                AutoCommands.aimSpeakerDynamic(true, 4500)
            ),
            
            AutoCommands.score(),

            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(1)),
                AutoCommands.followTrajectory(paths.get(2)),
                Commands.parallel(
                    AutoCommands.aimSpeakerDynamic(true, 5000),
                    AutoCommands.rotateToSpeaker()
                ),
                AutoCommands.score()
            ).until(() -> AutoGamepieces.isGone(4)),

            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(3)),
                AutoCommands.followTrajectory(paths.get(4)),
                Commands.parallel(
                    AutoCommands.aimSpeakerDynamic(true, 5000),
                    AutoCommands.rotateToSpeaker()
                ),
                AutoCommands.score()
            ).until(() -> AutoGamepieces.isGone(5)),

            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(5)),
                AutoCommands.SetPivotAngle(PivotSetpoints.stow),
                AutoCommands.followTrajectory(paths.get(6)),
                Commands.parallel(
                    AutoCommands.aimSpeakerDynamic(true, 5000),
                    AutoCommands.rotateToSpeaker()
                ),
                AutoCommands.score()
            )
        );
    }
}
