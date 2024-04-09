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

public class GDA_M32145RE {
    private static List<PathPlannerPath> paths;
    PathPlannerAuto auto;

    public static Command load(){
        paths = PathPlannerAuto.getPathGroupFromAutoFile("M GP 3214 trying to save time");

        return Commands.sequence(
            AutoCommands.initialize(4500),
            AutoCommands.SetPivotAngle(0.3),
            AutoCommands.followTrajectory(paths.get(0)),

            Commands.sequence(
            Commands.waitSeconds(0.2),
            Commands.race(AutoCommands.followTrajectory(paths.get(1)).andThen(Commands.waitSeconds(0.25)),
            AutoCommands.intake()),
            AutoCommands.followTrajectory(paths.get(2))).until(() -> AutoGamepieces.isGone(4)),

            Commands.sequence(
                Commands.race(AutoCommands.followTrajectory(paths.get(3)).andThen(Commands.waitSeconds(0.25),
                AutoCommands.intake())),
                AutoCommands.followTrajectory(paths.get(4))).until(() -> AutoGamepieces.isGone(5)));
        }
    }