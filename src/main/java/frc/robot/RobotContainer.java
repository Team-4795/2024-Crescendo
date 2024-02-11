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

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmFeedForwardCharacterization;
import frc.robot.commands.ScoreSpeaker;
import frc.robot.subsystems.MAXSwerve.*;
import frc.robot.StateManager.State;
import frc.robot.subsystems.Shooter.*;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.pivot.*;

import frc.robot.subsystems.intake.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
// import frc.robot.commands.TurnToSpeaker;

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
  private final Shooter shooter;
  private final Pivot pivot;
  private final Indexer indexer;
  private final Intake intake;
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
        // Real robot, instantiate hardware IO implementations
        drive = Drive.initialize(
            new GyroIOPigeon2(),
            new ModuleIOSparkMax(DriveConstants.kFrontLeftDrivingCanId, DriveConstants.kFrontLeftTurningCanId, DriveConstants.kFrontLeftChassisAngularOffset),
            new ModuleIOSparkMax(DriveConstants.kFrontRightDrivingCanId, DriveConstants.kFrontRightTurningCanId, DriveConstants.kFrontRightChassisAngularOffset),
            new ModuleIOSparkMax(DriveConstants.kRearLeftDrivingCanId, DriveConstants.kRearLeftTurningCanId, DriveConstants.kBackLeftChassisAngularOffset),
            new ModuleIOSparkMax(DriveConstants.kRearRightDrivingCanId, DriveConstants.kRearRightTurningCanId, DriveConstants.kBackRightChassisAngularOffset));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        intake = Intake.initialize(new IntakeIOSim());
        shooter = Shooter.initialize(new ShooterIOSim());
        pivot = Pivot.initialize(new PivotIOSim());
        indexer = Indexer.initialize(new IndexerIOSim());
        drive = Drive.initialize(
            new GyroIO() {},
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
        drive = Drive.initialize(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
        break;
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption("Arm Characterization", new ArmFeedForwardCharacterization(pivot, pivot::runVoltage, pivot::getVelocity, pivot::getPosition));

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
    drive.setDefaultCommand(new RunCommand(
        () -> drive.drive(
            -MathUtil.applyDeadband(OIConstants.driverController.getLeftY(), OIConstants.kAxisDeadband),
            -MathUtil.applyDeadband(OIConstants.driverController.getLeftX(), OIConstants.kAxisDeadband),
            MathUtil.applyDeadband(OIConstants.driverController.getRightX(), OIConstants.kAxisDeadband),
            true, true),
        drive));

    OIConstants.driverController.leftTrigger(0.5).whileTrue(new ScoreSpeaker());

    OIConstants.driverController.rightTrigger(0.5).whileTrue(Commands.startEnd(
      () -> shooter.setShootingSpeed(0.5),
      () -> shooter.setShootingSpeed(0), 
      shooter));

    // OIConstants.operatorController.rightTrigger(0.5).whileTrue(Commands.startEnd(
    //   () -> shooter.setShootingSpeed(-0.5), 
    //   () -> shooter.setShootingSpeed(0), 
    //   shooter));

    OIConstants.driverController.rightTrigger(0.5).whileTrue(Commands.startEnd(
      () -> indexer.setSpin(true), 
      () -> indexer.setSpin(false), 
      indexer));

    OIConstants.operatorController.povRight().onTrue(Commands.runOnce(() -> manager.setState(State.Stow)));
    OIConstants.operatorController.povLeft().onTrue(Commands.runOnce(() -> manager.setState(State.SourceIntake)));
    OIConstants.operatorController.povDown().onTrue(Commands.runOnce(() -> manager.setState(State.GroundIntake)));
    OIConstants.operatorController.povUp().onTrue(Commands.runOnce(() -> manager.setState(State.ScoreAmp)));

    OIConstants.operatorController.a().whileTrue(Commands.startEnd(
        () -> intake.setOverride(true),
        () -> intake.setOverride(false),
        intake));
        
    OIConstants.operatorController.y().onTrue(Commands.runOnce(() -> indexer.reverse()));
    OIConstants.operatorController.b().whileTrue(Commands.startEnd(
        () -> indexer.setOverride(true),
        () -> indexer.setOverride(false),
        indexer
      )
    );
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