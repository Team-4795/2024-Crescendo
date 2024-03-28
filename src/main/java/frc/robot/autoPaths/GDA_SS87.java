package frc.robot.autoPaths;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoCommands;

public class GDA_SS87 {
    private static List<PathPlannerPath> paths;
    PathPlannerAuto auto;

    public static Command load(){
        paths = PathPlannerAuto.getPathGroupFromAutoFile("GDA SS GP 87");

        return Commands.sequence(
            AutoCommands.rotateToSpeaker(),
            AutoCommands.aimSpeakerDynamic(true),
            AutoCommands.score(),

            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(0)),
                Commands.waitSeconds(0.2),
                    Commands.sequence(
                        //AutoCommands.SetPivotAngle(.129),
                        AutoCommands.followTrajectory(paths.get(1))
                    ),
            AutoCommands.aimSpeakerDynamic(true),
            AutoCommands.score()
            ).until(() -> AutoGamepieces.isGone(8)),


            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(2)),
                Commands.waitSeconds(0.2),
                    Commands.sequence(
                        //AutoCommands.SetPivotAngle(.129),
                        AutoCommands.followTrajectory(paths.get(3))
                    ),
            AutoCommands.aimSpeakerDynamic(true),
            AutoCommands.score()
            ).until(() -> AutoGamepieces.isGone(7)),

            AutoCommands.intakeTrajectory(paths.get(4)));
    }
}