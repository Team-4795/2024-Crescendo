package frc.robot.util;

import java.util.Collections;

import com.ctre.phoenix6.configs.jni.ConfigJNI;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Mode;
import frc.robot.Constants;
import frc.robot.autoPaths.AutoGamepieces;
import frc.robot.commands.AutoCommands;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.vision.intakeCam.IntakeCamVision;

public class NamedCommandManager {
    public static void registerAll() {
        NamedCommands.registerCommand("Score", AutoCommands.score());

        NamedCommands.registerCommand("Align", AutoCommands.SetPivotAngle(0.2)); // change later

        NamedCommands.registerCommand("AlignGP1", AutoCommands.SetPivotAngle(0.275)); // change later

        //NamedCommands.registerCommand("AlignGP1", AutoCommands.setPivotAndShooter(0.275, 4000)); // change later

        NamedCommands.registerCommand("AlignCommunityLine", AutoCommands.setPivotAndShooter(0.25, 4000)); // change later

        NamedCommands.registerCommand("PoopOut", AutoCommands.setPivotAndShooter(0.25, 2000)); // change later

        NamedCommands.registerCommand("AlignGP2", AutoCommands.setPivotAndShooter(0.27, 4000)); // change later

        NamedCommands.registerCommand("AlignCloseGP", AutoCommands.setPivotAndShooter(0.3, 4500)); // change later

        NamedCommands.registerCommand(".29", AutoCommands.setPivotAndShooter(0.29, 4500)); // change later

        NamedCommands.registerCommand("AlignGP3", AutoCommands.setPivotAndShooter(0.21, 4000)); // change later

        NamedCommands.registerCommand("Align Far Source", AutoCommands.setPivotAndShooter(0.135, 5000));

        NamedCommands.registerCommand("SetIntakeAngle", AutoCommands.SetPivotAngle(0.3));

        NamedCommands.registerCommand("Align Under Stage", AutoCommands.setPivotAndShooter(0.17, 4500));

        NamedCommands.registerCommand("Stow", AutoCommands.SetPivotAngle(0.08));

        NamedCommands.registerCommand("AlignClose", AutoCommands.SetPivotAngle(0.6)); // change later

        NamedCommands.registerCommand("Initialize", AutoCommands.initialize(4500));

        NamedCommands.registerCommand("RunEverything", AutoCommands.runEverything(4500));

        NamedCommands.registerCommand("RunEverything 5k", AutoCommands.runEverything(5000));

        NamedCommands.registerCommand("Stop Shooting", AutoCommands.stopShooting());

        NamedCommands.registerCommand("SensePiece", AutoCommands.sensingPiece());

        NamedCommands.registerCommand("Intake", AutoCommands.intake());

        NamedCommands.registerCommand("Run Intake", AutoCommands.intakeWithoutPivot());

        NamedCommands.registerCommand("SetIntakePose", AutoCommands.SetPivotAngle(0.6));

        NamedCommands.registerCommand("0.24", AutoCommands.setPivotAndShooter(0.24, 4500));

        NamedCommands.registerCommand("Align Speaker", AutoCommands.rotateToSpeaker());

        NamedCommands.registerCommand("VisionAlign", AutoCommands.aimSpeakerDynamic(true, 4500));

        NamedCommands.registerCommand("VisionAlign 5k", AutoCommands.aimSpeakerDynamic(true, 5000));
                    
        NamedCommands.registerCommand("Align Wing Amp", AutoCommands.setPivotAndShooter(0.133, 4500));

        NamedCommands.registerCommand("Align Wing Amp Blue", AutoCommands.setPivotAndShooter(0.1295, 4500));

        NamedCommands.registerCommand("Detect Note 4", detectNote(4, true));

        NamedCommands.registerCommand("Detect Note 5", detectNote(5, true));

        NamedCommands.registerCommand("Detect Note 6", detectNote(6, true));

        NamedCommands.registerCommand("Detect Note 7", detectNote(7, true));

        NamedCommands.registerCommand("Detect Note 8", detectNote(8, true));

    }

    private static Command detectNote(int note, boolean simDetect) {
        return Commands.parallel(
            Commands.print("Event Marker Executed For Note " + note),
            Commands.either(
                Commands.sequence(
                    Commands.waitUntil(() -> IntakeCamVision.getInstance().getDistanceToNote(note) < 2.5),
                    Commands.parallel(
                        Commands.runOnce(() -> AutoGamepieces.setNoteGone(note)),
                        Commands.print("Note " + note + " not detected")
                    ).onlyIf(() -> !simDetect)
                ), 
                Commands.run(() -> {
                    if(IntakeCamVision.getInstance().getDistanceToNote(note) < 5){
                        boolean noteSeen;
                        AutoGamepieces.setNote(note, noteSeen = IntakeCamVision.getInstance().isNoteInFront(note));
                        System.out.println("Note " + note + " seen?: " + noteSeen);
                    }
                }).until(() -> IntakeCamVision.getInstance().getDistanceToNote(note) < 2.5)
                  .finallyDo(() -> AutoGamepieces.updateNotes(note)),
                () -> Constants.currentMode == Mode.SIM)
        );
    }
}
