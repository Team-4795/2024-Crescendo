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

public class GDA_AS456Move {
    private static List<PathPlannerPath> paths;
    PathPlannerAuto auto;

    public static Command load(){
        paths = PathPlannerAuto.getPathGroupFromAutoFile("AS GP 456");
        IntakeCamVision.getInstance().setTargetComparator(PhotonTargetSortMode.Centermost);
        
        return Commands.sequence(
            Commands.parallel(
                AutoCommands.rotateToSpeaker(),
                AutoCommands.aimSpeakerDynamic(true, 4500)
            ),
            AutoCommands.score(),

            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(0)),
                    Commands.parallel(
                        AutoCommands.setPivotAndShooter(0.17, 5000), //change this later
                        AutoCommands.followTrajectory(paths.get(1)),
                        Commands.sequence(Commands.waitSeconds(1.5), AutoCommands.score())
                )).until(() -> AutoGamepieces.isGone(4)),

            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(2)),
                AutoCommands.setPivotAndShooter(0.17, 5000),
                Commands.parallel(
                        AutoCommands.followTrajectory(paths.get(3)), 
                        Commands.sequence(Commands.waitSeconds(1.4), AutoCommands.score()) //change this later
                    )).until(() -> AutoGamepieces.isGone(5)),

            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(4)),
                AutoCommands.SetPivotAngle(PivotSetpoints.stow),
                AutoCommands.followTrajectory(paths.get(5)),
                AutoCommands.aimSpeakerDynamic(true, 5000)
            ),
            AutoCommands.score()
        );
    }
}
