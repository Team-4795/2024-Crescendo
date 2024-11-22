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
import frc.robot.subsystems.indexer.*;

public class Adaptive_Test {
    private static List<PathPlannerPath> paths;
    PathPlannerAuto auto;

    public static Command load(){
        paths = PathPlannerAuto.getPathGroupFromAutoFile("Adaptive Testing");
        IntakeCamVision.getInstance().setTargetComparator(PhotonTargetSortMode.Centermost);
        
        return Commands.sequence(
            Commands.parallel(
                AutoCommands.rotateToSpeaker(),
                AutoCommands.aimSpeakerDynamic(true, 4500)
            ),
            AutoCommands.intakeTrajectory(paths.get(0)),
            Commands.sequence(
                    AutoCommands.followTrajectory(paths.get(1)), 
                    Commands.parallel(AutoCommands.rotateToSpeaker(),
                    AutoCommands.aimSpeakerDynamic(true, 5000))).onlyIf(() -> Indexer.getInstance().isStoring()),
            AutoCommands.intakeTrajectory(paths.get(2)),
            Commands.sequence(AutoCommands.followTrajectory(paths.get(3)),
            Commands.parallel(AutoCommands.rotateToSpeaker(),
                AutoCommands.aimSpeakerDynamic(true, 4500))).onlyIf(() -> Indexer.getInstance().isStoring()),
            AutoCommands.intakeTrajectory(paths.get(4)));
    }
}
