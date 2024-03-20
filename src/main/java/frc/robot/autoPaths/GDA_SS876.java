package frc.robot.autoPaths;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoCommands;

public class GDA_SS876 {
    private static List<PathPlannerPath> paths;
    PathPlannerAuto auto;

    public static Command load(){
        paths = PathPlannerAuto.getPathGroupFromAutoFile("GDA SS GP 876");
        
        return Commands.sequence(
            AutoCommands.followTrajectory(paths.get(0)).until(AutoPath::next),
            AutoCommands.followTrajectory(paths.get(2)).until(AutoPath::next2),
            AutoCommands.followTrajectory(paths.get(4))
        );
    }
}
