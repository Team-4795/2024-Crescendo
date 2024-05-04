package frc.robot.autoPaths;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoCommands;

public class GDA_SS87Move {
    private static List<PathPlannerPath> paths;
    PathPlannerAuto auto;

    public static Command load(){
        paths = PathPlannerAuto.getPathGroupFromAutoFile("Shoot on the Move SS GP 876");

        return Commands.sequence(

            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(0)),
                    Commands.parallel(
                        AutoCommands.SetPivotAngle(.129), //change later
                        AutoCommands.followTrajectory(paths.get(1)),
                        Commands.sequence(Commands.waitSeconds(1.6), 
                        AutoCommands.score())
                    )).until(() -> AutoGamepieces.isGone(8)),


            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(2)),
                    Commands.parallel(
                        AutoCommands.SetPivotAngle(.129), //change later
                        AutoCommands.followTrajectory(paths.get(3)),
                        Commands.sequence(Commands.waitSeconds(1.9), AutoCommands.score())
                    )).until(() -> AutoGamepieces.isGone(7)),
           
            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(4)),
                        Commands.parallel(
                        AutoCommands.SetPivotAngle(.08),
                        AutoCommands.followTrajectory(paths.get(5))), 
                        AutoCommands.setPivotAndShooter(.129, 5000), //change this later
                        Commands.waitSeconds(0.2),
                        AutoCommands.score()));
    }
}