package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;

import frc.robot.Constants;
import frc.robot.subsystems.MAXSwerve.Drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToAmp {
        // change these later to allign with amp
        private static final boolean isRed = DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get().equals(Alliance.Red);

        private static Pose2d targetPose = (isRed ? Constants.PathFindingConstants.redAmp
                        : Constants.PathFindingConstants.blueAmp);

        // change later
        private static PathConstraints constraints = new PathConstraints(
                        1, 4,
                        Units.degreesToRadians(540), Units.degreesToRadians(720));

// Command pathfindingCommand = new PathfindHolonomic(
//         targetPose,
//         constraints,
//         0.0, // Goal end velocity in m/s. Optional
//         Drive.getInstance()::getPose,
//         Drive.getInstance()::getRobotRelativeSpeeds,
//         Drive.getInstance()::driveRobotRelative,
//         Constants.Swerve.pathFollowingConfig, // HolonomicPathFollwerConfig, see the API or "Follow a single path" example for more info
//         0.0, // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate. Optional
//         Drive.getInstance() // Reference to drive subsystem to set requirements
// );
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
}
