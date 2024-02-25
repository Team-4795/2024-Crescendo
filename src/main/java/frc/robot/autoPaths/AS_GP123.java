package frc.robot.autoPaths;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoCommands;

public class AS_GP123 extends AutoPath{
    public Command load(AutoCommands autoCommands) {
        PathPlannerPath path1 = PathPlannerPath.fromPathFile("AS GP123 P1");
        
        return Commands.sequence(
            AutoCommands.initialize(1),
            AutoCommands.resetOdometry(path1.getPreviewStartingHolonomicPose()),
            AutoCommands.score(),
            AutoCommands.alignTrajectory("AS GP123 P1", AutoConstants.closePivotSetpoint),
            AutoCommands.score(),
            AutoCommands.alignTrajectory("AS GP123 P2", AutoConstants.closePivotSetpoint),
            AutoCommands.score(),
            AutoCommands.alignTrajectory("AS GP123 P3", AutoConstants.closePivotSetpoint),
            AutoCommands.score()
        );
    }
}

