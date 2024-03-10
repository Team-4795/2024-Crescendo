package frc.robot.util;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ShooterSetpoints;
import frc.robot.commands.AutoCommands;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.Shooter.Shooter;

public class NamedCommandManager {
    public static void registerAll() {
        NamedCommands.registerCommand("Score", AutoCommands.score());

        NamedCommands.registerCommand("Align", AutoCommands.SetPivotAngle(0.2)); // change later

        NamedCommands.registerCommand("AlignGP1", AutoCommands.setPivotAndShooter(0.25)); // change later

        NamedCommands.registerCommand("AlignCommunityLine", AutoCommands.setPivotAndShooter(0.25)); // change later


        NamedCommands.registerCommand("AlignGP2", AutoCommands.setPivotAndShooter(0.2425)); // change later

        NamedCommands.registerCommand("AlignGP3", AutoCommands.setPivotAndShooter(0.2)); // change later

        NamedCommands.registerCommand("AlignClose", AutoCommands.SetPivotAngle(0.58)); // change later

        NamedCommands.registerCommand("Align Subwoofer", AutoCommands.SetPivotAngle(0.58));

        NamedCommands.registerCommand("Initialize", AutoCommands.initialize(1));

        NamedCommands.registerCommand("RunEverything", AutoCommands.runEverything(1));

        NamedCommands.registerCommand("StopIndexer", AutoCommands.runIndexer(0));

        NamedCommands.registerCommand("SensePiece", AutoCommands.sensingPiece());

        NamedCommands.registerCommand("Intake", AutoCommands.intake());

        NamedCommands.registerCommand("Jostle Pivot", AutoCommands.SetPivotAngle(0.08));

        NamedCommands.registerCommand("SetIntakePose", AutoCommands.SetPivotAngle(0.6));

        NamedCommands.registerCommand("VisionAlign",
                AutoCommands.aimSpeakerDynamic().withTimeout(0.5)
                .alongWith(Commands.runOnce(
                        () -> Shooter.getInstance().setShootingSpeedRPM(ShooterSetpoints.speakerTop, ShooterSetpoints.speakerBottom))));
    }
}
