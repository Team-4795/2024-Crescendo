package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;

import frc.robot.Constants;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.leds.LEDs;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToAmp {
	// Load the path we want to pathfind to and follow
	static PathPlannerPath path = PathPlannerPath.fromPathFile("Align To Amp");
	
	// Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
	static PathConstraints constraints = new PathConstraints(
        2, 
        5, 
        Units.degreesToRadians(540), 
        Units.degreesToRadians(720));
	
	// Since AutoBuilder is configured, we can use it to build pathfinding commands
	public static Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints, 3.0)
        .raceWith(LEDs.getInstance().pathfinding());
}
