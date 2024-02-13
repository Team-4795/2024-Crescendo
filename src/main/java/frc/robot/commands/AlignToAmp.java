package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;


public class AlignToAmp{
//change these later to allign with amp 
final boolean isRed = 
        DriverStation.getAlliance().isPresent()
         && DriverStation.getAlliance().get().equals(Alliance.Red);

Pose2d targetPose = (isRed ? Constants.PathFindingConstants.redAmp : Constants.PathFindingConstants.blueAmp);

// change later
PathConstraints constraints = new PathConstraints(
        3, 4,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
    
// Since AutoBuilder is configured, we can use it to build pathfinding commands
public Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        3, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
);
}
