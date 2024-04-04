package frc.robot.autoPaths;

import java.util.List;

import org.photonvision.PhotonTargetSortMode;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoCommands;
import frc.robot.subsystems.vision.intakeCam.IntakeCamVision;

public class GDA_M32145 {
    private static List<PathPlannerPath> paths;
    PathPlannerAuto auto;

    public static Command load(){
        paths = PathPlannerAuto.getPathGroupFromAutoFile("M GP 3214");
        IntakeCamVision.getInstance().setTargetComparator(PhotonTargetSortMode.Centermost);


        return Commands.sequence(
        AutoCommands.initialize(4500),
        AutoCommands.SetPivotAngle(0.33),

        Commands.sequence(
            AutoCommands.followTrajectory(paths.get(0))),
            Commands.waitSeconds(0.4),

            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(1)),
                Commands.deadline( 
                    Commands.sequence(
                        //AutoCommands.SetPivotAngle(.129),
                        AutoCommands.followTrajectory(paths.get(2)), 
                        Commands.waitSeconds(0.2)
                    ), AutoCommands.aimSpeakerDynamic(false, 5000)),
                AutoCommands.score()
            ),//.until(() -> AutoGamepieces.isGone(4)),

            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(3)),
                Commands.deadline(
                    Commands.sequence(
                        AutoCommands.followTrajectory(paths.get(4)), 
                        Commands.waitSeconds(0.3)
                    )), AutoCommands.aimSpeakerDynamic(false, 5000)),

                AutoCommands.score()
            );//.until(() -> AutoGamepieces.isGone(5));

    }
}