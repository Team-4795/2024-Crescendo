package frc.robot.util;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ShooterSetpoints;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.autoPaths.AutoGamepieces;
import frc.robot.commands.AutoCommands;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.vision.intakeCam.IntakeCamVision;

public class NamedCommandManager {
    public static void registerAll() {
        NamedCommands.registerCommand("Score", AutoCommands.score());

        NamedCommands.registerCommand("Align", AutoCommands.SetPivotAngle(0.2)); // change later

        NamedCommands.registerCommand("AlignGP1", AutoCommands.setPivotAndShooter(0.25)); // change later

        NamedCommands.registerCommand("AlignCommunityLine", AutoCommands.setPivotAndShooter(0.25)); // change later

        NamedCommands.registerCommand("AlignGP2", AutoCommands.setPivotAndShooter(0.2425)); // change later

        NamedCommands.registerCommand("AlignGP3", AutoCommands.setPivotAndShooter(0.2)); // change later

        NamedCommands.registerCommand("Align Far Source", AutoCommands.setPivotAndShooter(0.1115));

        NamedCommands.registerCommand("Align Under Stage", AutoCommands.setPivotAndShooter(0.16));

        NamedCommands.registerCommand("Stow", AutoCommands.SetPivotAngle(0.08));

        NamedCommands.registerCommand("AlignClose", AutoCommands.SetPivotAngle(0.58)); // change later

        NamedCommands.registerCommand("Initialize", AutoCommands.initialize(1));

        NamedCommands.registerCommand("RunEverything", AutoCommands.runEverything(1));

        NamedCommands.registerCommand("StopIndexer", AutoCommands.runIndexer(0));

        NamedCommands.registerCommand("SensePiece", AutoCommands.sensingPiece());

        NamedCommands.registerCommand("Intake", AutoCommands.intake());

        NamedCommands.registerCommand("Run Intake", AutoCommands.intakeWithoutPivot());

        NamedCommands.registerCommand("SetIntakePose", AutoCommands.SetPivotAngle(0.6));

        NamedCommands.registerCommand("Align Speaker", AutoCommands.rotateToSpeaker());

        NamedCommands.registerCommand("VisionAlign", AutoCommands.aimSpeakerDynamic(true));
                    
        NamedCommands.registerCommand("Align Wing Amp", AutoCommands.setPivotAndShooter(0.133));

        NamedCommands.registerCommand("Align Wing Amp Blue", AutoCommands.setPivotAndShooter(0.1295));

        NamedCommands.registerCommand("Detect Note 4", detectNote(4, true));

        NamedCommands.registerCommand("Detect Note 5", detectNote(5, true));

        NamedCommands.registerCommand("Detect Note 6", detectNote(6, true));

        NamedCommands.registerCommand("Detect Note 7", detectNote(7, true));

        NamedCommands.registerCommand("Detect Note 8", detectNote(8, true));
    }

    private static Command detectNote(int note, boolean simDetect) {
        return Commands.parallel(
            Commands.print("Event Marker Executed"),
            Commands.either(
                Commands.runOnce(() -> AutoGamepieces.setNoteGone(note)).onlyIf(() -> !simDetect), 
                Commands.runOnce(() -> AutoGamepieces.setNoteGone(note)).onlyIf(IntakeCamVision.getInstance()::isNoteInFront), 
                () -> Constants.currentMode == Mode.SIM)
        );
    }
}
