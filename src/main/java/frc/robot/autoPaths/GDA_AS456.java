package frc.robot.autoPaths;

import java.util.List;

import org.photonvision.PhotonTargetSortMode;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.Constants.PivotSetpoints;
import frc.robot.commands.AutoCommands;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.vision.intakeCam.IntakeCamVision;

public class GDA_AS456 {
    private static List<PathPlannerPath> paths;
    PathPlannerAuto auto;

    public static Command load(){

        if (Drive.getInstance().getPose().getX() == 0 && Drive.getInstance().getPose().getY() == 0) {
            DriverStation.getAlliance().ifPresent((alliance) -> {
                if(alliance == Alliance.Blue){
                    Drive.getInstance().resetOdometry(new Pose2d(1.4, 7.37, new Rotation2d(0)));
                } else {
                    Drive.getInstance().resetOdometry(new Pose2d(15.09, 7.37, new Rotation2d(Math.PI)));
                }
            });
        }
        paths = PathPlannerAuto.getPathGroupFromAutoFile("AS GP 456");
        PathPlannerPath note5 = PathPlannerPath.fromPathFile("AS GP456 P2.5");
        PathPlannerPath note6 = PathPlannerPath.fromPathFile("AS GP456 P4.5");
        IntakeCamVision.getInstance().setTargetComparator(PhotonTargetSortMode.Centermost);
        
        return Commands.sequence(
            // AutoCommands.followTrajectory(paths.get(0)),

            Commands.parallel(
                AutoCommands.rotateToSpeaker(),
                AutoCommands.aimSpeakerDynamic(true, 4500)
            ),
            
            AutoCommands.score(),

            Commands.sequence(
                AutoCommands.intakeTrajectory(paths.get(0)),
                Commands.runOnce(() -> AutoGamepieces.setNoteGone(4)).onlyIf(() -> noNoteDetected(true)),
                AutoCommands.followTrajectory(paths.get(1)),
                Commands.parallel(
                    AutoCommands.aimSpeakerDynamic(true, 5000),
                    AutoCommands.rotateToSpeaker()
                ),
                AutoCommands.score()
            ).until(() -> AutoGamepieces.isGone(4)),

            Commands.sequence(
                Commands.either(
                    AutoCommands.intakeTrajectory(note5), 
                    AutoCommands.intakeTrajectory(paths.get(2)), 
                    () -> noNoteDetected()),
                Commands.runOnce(() -> AutoGamepieces.setNoteGone(5)).onlyIf(() -> noNoteDetected(false)),
                AutoCommands.followTrajectory(paths.get(3)),
                Commands.parallel(
                    AutoCommands.aimSpeakerDynamic(true, 5000),
                    AutoCommands.rotateToSpeaker()
                ),
                AutoCommands.score()
            ).until(() -> AutoGamepieces.isGone(5)),

            Commands.sequence(
                Commands.either(
                    AutoCommands.intakeTrajectory(note6), 
                    AutoCommands.intakeTrajectory(paths.get(4)), 
                    () -> noNoteDetected()),
                AutoCommands.SetPivotAngle(PivotSetpoints.stow),
                AutoCommands.followTrajectory(paths.get(5)),
                Commands.parallel(
                    AutoCommands.aimSpeakerDynamic(true, 5000),
                    AutoCommands.rotateToSpeaker()
                ),
                AutoCommands.score()
            )
        );
    }

    private static boolean noNoteDetected(boolean simDetect){
        if(Robot.isSimulation()){
            Indexer.getInstance().setIntakeAuto(simDetect);
        }
        return !Indexer.getInstance().getIntakeDetected();
    }

    private static boolean noNoteDetected(){
        return !Indexer.getInstance().getIntakeDetected();
    }
}
