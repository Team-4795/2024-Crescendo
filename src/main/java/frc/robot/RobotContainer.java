// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterSetpoints;
import frc.robot.StateManager.State;
import frc.robot.subsystems.MAXSwerve.*;
import frc.robot.subsystems.Shooter.*;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.pivot.*;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.util.NoteVisualizer;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AlignToAmp;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.RainbowCommand;
import frc.robot.commands.AlignHeading;
import frc.robot.commands.AlignSpeaker;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Shooter shooter;
  private final Pivot pivot;
  private final Indexer indexer;
  private final Intake intake;
  private LEDs leds;

  // Managers
  private final StateManager manager = StateManager.getInstance();

  LoggedDashboardChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        intake = Intake.initialize(new IntakeIOReal());
        shooter = Shooter.initialize(new ShooterIOReal());
        pivot = Pivot.initialize(new PivotIOReal());
        indexer = Indexer.initialize(new IndexerIOReal());
        vision = Vision.initialize(new VisionIOReal());
        drive = Drive.initialize(
            new GyroIOPigeon2(),
            new ModuleIOSparkMax(DriveConstants.kFrontLeftDrivingCanId, DriveConstants.kFrontLeftTurningCanId,
                DriveConstants.kFrontLeftChassisAngularOffset),
            new ModuleIOSparkMax(DriveConstants.kFrontRightDrivingCanId, DriveConstants.kFrontRightTurningCanId,
                DriveConstants.kFrontRightChassisAngularOffset),
            new ModuleIOSparkMax(DriveConstants.kRearLeftDrivingCanId, DriveConstants.kRearLeftTurningCanId,
                DriveConstants.kBackLeftChassisAngularOffset),
            new ModuleIOSparkMax(DriveConstants.kRearRightDrivingCanId, DriveConstants.kRearRightTurningCanId,
                DriveConstants.kBackRightChassisAngularOffset));
        leds = LEDs.getInstance();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        intake = Intake.initialize(new IntakeIOSim());
        shooter = Shooter.initialize(new ShooterIOSim());
        pivot = Pivot.initialize(new PivotIOSim());
        indexer = Indexer.initialize(new IndexerIOSim());
        vision = Vision.initialize(new VisionIO() {});
        drive = Drive.initialize(
            new GyroIOSim(),
            new ModuleIOSim(DriveConstants.kFrontLeftChassisAngularOffset),
            new ModuleIOSim(DriveConstants.kFrontRightChassisAngularOffset),
            new ModuleIOSim(DriveConstants.kBackLeftChassisAngularOffset),
            new ModuleIOSim(DriveConstants.kBackRightChassisAngularOffset));

        break;

      default:
        intake = Intake.initialize(new IntakeIO() {});
        shooter = Shooter.initialize(new ShooterIO() {});
        pivot = Pivot.initialize(new PivotIO() {});
        indexer = Indexer.initialize(new IndexerIO() {});
        vision = Vision.initialize(new VisionIO() {});
        drive = Drive.initialize(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
        break;
    }
    NamedCommands.registerCommand("Score", AutoCommands.score());
    NamedCommands.registerCommand("Align", AutoCommands.SetPivotAngle(0.2)); //change later
    NamedCommands.registerCommand("AlignClose", AutoCommands.SetPivotAngle(0.6)); //change later
    NamedCommands.registerCommand("Align Subwoofer", AutoCommands.SetPivotAngle(0.6));
    NamedCommands.registerCommand("Initialize", AutoCommands.initialize(1));
    NamedCommands.registerCommand("RunEverything", AutoCommands.runEverything(1));
    NamedCommands.registerCommand("StopIndexer", AutoCommands.runIndexer(0));
    NamedCommands.registerCommand("SensePiece", AutoCommands.sensingPiece());
    NamedCommands.registerCommand("Intake", AutoCommands.intake());
    NamedCommands.registerCommand("Jostle Pivot", AutoCommands.SetPivotAngle(0.08));
    NamedCommands.registerCommand("SetIntakePose", AutoCommands.SetPivotAngle(0.6));
    NamedCommands.registerCommand("VisionAlign", AutoCommands.aimSpeakerDynamic().alongWith(Commands.runOnce(() -> shooter.setShootingSpeedRPM(ShooterSetpoints.speakerTop, ShooterSetpoints.speakerBottom))).withTimeout(1));


    manager.setState(State.Init);

    NoteVisualizer.setRobotPoseSupplier(drive::getPose);
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser("TEST - AS GP123"));



    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    Trigger isReady = new Trigger(() -> pivot.atSetpoint() && shooter.atSetpoint());

    drive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> drive.drive(
                -MathUtil.applyDeadband(OIConstants.driverController.getLeftY(), OIConstants.kAxisDeadband),
                -MathUtil.applyDeadband(OIConstants.driverController.getLeftX(), OIConstants.kAxisDeadband),
                -MathUtil.applyDeadband(OIConstants.driverController.getRightX(), OIConstants.kAxisDeadband),
                true, true),
            drive));

    // Zero drive heading
    OIConstants.driverController.rightBumper().onTrue(new InstantCommand(drive::zeroHeading));

    // Auto align
    OIConstants.driverController.povUp().or(OIConstants.driverController.leftBumper())
      .whileTrue(new AlignSpeaker());

    OIConstants.driverController.rightTrigger(0.3).or(OIConstants.driverController.leftTrigger(0.3))
        .whileTrue(
            Commands.sequence(
                indexer.reverse().withTimeout(0.05),
                indexer.forwards()));

    new Trigger(() -> Math.abs(OIConstants.operatorController.getLeftY()) > 0.15)
      .whileTrue(
        new RainbowCommand(() -> MathUtil.applyDeadband(OIConstants.operatorController.getLeftY(), 0.15)));

    // Auto drive align
    OIConstants.driverController.povDown().whileTrue(AlignToAmp.pathfindingCommand);
    OIConstants.driverController.povRight().onTrue(Commands.runOnce(() -> manager.setState(State.Stow)));
    OIConstants.driverController.povLeft().onTrue(Commands.runOnce(() -> pivot.toggleAutoAim()));

    OIConstants.driverController.y().whileTrue(AlignHeading.align(0));
    OIConstants.driverController.x().whileTrue(AlignHeading.align(Units.degreesToRadians(90)));
    OIConstants.driverController.a().whileTrue(AlignHeading.align(Units.degreesToRadians(180)));
    OIConstants.driverController.b().whileTrue(AlignHeading.align(Units.degreesToRadians(270)));

    // Speaker aim and rev up
    OIConstants.operatorController.leftBumper()
      .whileTrue(pivot.aimSpeakerDynamic().alongWith(shooter.revSpeaker()));

    // OIConstants.operatorController.leftBumper().whileTrue(
    //   shooter.revSpeaker()
    // );
    // Amp aim and rev up
    OIConstants.operatorController.rightBumper().whileTrue(
        pivot.aimAmp().alongWith(shooter.revAmp()));

    OIConstants.operatorController.povUp().onTrue(
        Commands.parallel(
            shooter.slowReverse(),
            indexer.slowReverse(),
            pivot.aimSource()));

    // Intake
    OIConstants.operatorController.povDown().whileTrue(
        Commands.parallel(
            pivot.aimIntake(),
            intake.intake(),
            indexer.forwards())
        .until(indexer::isStoring)
        .andThen(Commands.parallel(
          rumbleCommand(0.5).withTimeout(0.5),
          indexer.reverse().withTimeout(0.1))
        )
    );

    // Slow reverse
    OIConstants.operatorController.a().whileTrue(
            Commands.parallel(
                indexer.slowReverse(),
                shooter.slowReverse()));

    // Full reverse
    OIConstants.operatorController.b().whileTrue(
        Commands.parallel(
            intake.reverse(),
            indexer.reverse(),
            shooter.reverse()));

    // Override storing (flips it)
    OIConstants.operatorController.x().whileTrue(indexer.overrideStoring());

    // Other reverse
    OIConstants.operatorController.y().whileTrue(
        Commands.parallel(
            intake.slowReverse(),
            indexer.forwards()));

  }

  private void setBothRumble(double amount) {
    OIConstants.driverController.getHID().setRumble(RumbleType.kBothRumble, amount);
    OIConstants.operatorController.getHID().setRumble(RumbleType.kBothRumble, amount);
  }

  public Command rumbleCommand(double amount) {
    // return Commands.run(() -> setBothRumble(amount)).finallyDo(() ->
    // setBothRumble(0));
    return Commands.startEnd(() -> setBothRumble(amount), () -> setBothRumble(0));
  }

  public void teleopInit() {
    manager.setState(State.Init);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}