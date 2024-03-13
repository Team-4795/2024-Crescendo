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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.Constants.OIConstants;
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
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.NamedCommandManager;
import frc.robot.util.NoteVisualizer;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AlignToGamepiece;
import frc.robot.commands.ArmFeedForwardCharacterization;
import frc.robot.commands.RainbowCommand;
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
        vision = Vision.initialize(new VisionIOSim());
        drive = Drive.initialize(
            new GyroIOSim(),
            new ModuleIOSim(DriveConstants.kFrontLeftChassisAngularOffset),
            new ModuleIOSim(DriveConstants.kFrontRightChassisAngularOffset),
            new ModuleIOSim(DriveConstants.kBackLeftChassisAngularOffset),
            new ModuleIOSim(DriveConstants.kBackRightChassisAngularOffset));
        leds = LEDs.getInstance();

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

    NamedCommandManager.registerAll();
    NoteVisualizer.setPivotPoseSupplier(pivot::getPose);
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser("SS GP 876"));


    autoChooser.addOption("Pivot SysIs (Quasistatic Forward)", pivot.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Pivot SysIs (Quasistatic Reverse)", pivot.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Pivot SysIs (Dynamic Forward)", pivot.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Pivot SysIs (Dynamic Reverse)", pivot.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Pivot Model", new ArmFeedForwardCharacterization(pivot, (volts) -> pivot.runVoltage(volts), () -> pivot.getVelocity(), () -> pivot.getPosition(), (x) -> 0.0));

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
    Trigger timeRumble = new Trigger(() -> between(DriverStation.getMatchTime(), 19, 21) || between(DriverStation.getMatchTime(), 39, 41));
    Trigger continuousRumble = new Trigger(() -> DriverStation.getMatchTime() <= 5);
    Trigger isReady = new Trigger(() -> pivot.atSetpoint() && shooter.atSetpoint() && drive.isAtTarget());

    // Zero drive heading
    OIConstants.driverController.x().whileTrue(new AlignToGamepiece());

    //Align Amp / Speaker
    OIConstants.driverController.leftBumper().whileTrue(
        Commands.either(
          new AlignSpeaker().alongWith(shooter.rev()).alongWith(pivot.aim()), 
          drive.AutoAlignAmp().alongWith(leds.pathfinding()), 
          () -> StateManager.getState() == State.SPEAKER
        )
    );

    //Shoot
    OIConstants.driverController.rightTrigger(0.3)
      .whileTrue(indexer.forwards())
      .onTrue(NoteVisualizer.shoot());
    
    //Drive robot relative
    OIConstants.driverController.leftTrigger(0.3)
      .onTrue(Commands.runOnce(() -> drive.setFieldRelative(false)))
      .onFalse(Commands.runOnce(() -> drive.setFieldRelative(true)));

    OIConstants.driverController.povLeft().onTrue(Commands.runOnce(() -> {
      pivot.toggleAutoAim();
      leds.toggleYellow();
    }));

    OIConstants.driverController.a().whileTrue(Commands.runOnce(drive::zeroHeading));
    OIConstants.driverController.b().whileTrue(drive.AutoAlignAmp());
    // Speaker aim and rev up
    OIConstants.operatorController.leftBumper().onTrue(Commands.runOnce(() -> StateManager.setState(State.SPEAKER)));
      
    // Amp aim and rev up
    OIConstants.operatorController.rightBumper().onTrue(Commands.runOnce(() -> StateManager.setState(State.AMP)));

    //Source Intake
    OIConstants.operatorController.povUp().onTrue(
        Commands.parallel(
            shooter.slowReverse(),
            indexer.slowReverse(),
            pivot.aimSource()));

    //Ground Intake
    OIConstants.operatorController.povDown().whileTrue(
        Commands.parallel(
            pivot.aimIntake(),
            intake.intake(),
            indexer.forwards())
        .until(indexer::isStoring)
        .andThen(Commands.parallel(
          rumbleCommand(0.5).withTimeout(0.5),
          indexer.reverse().withTimeout(0.05))
        ).onlyIf(() -> !intake.getIdleMode())
    );

    OIConstants.operatorController.povLeft().onTrue(
      Commands.sequence(
        Commands.runOnce(() -> intake.setIdleMode(!intake.getIdleMode())),
        Commands.runOnce(() -> intake.setIntakeSpeed(0))
      )  
    );

    // Slow reverse tower
    // OIConstants.operatorController.a().whileTrue(
    //         Commands.parallel(
    //             indexer.slowReverse(),
    //             shooter.slowReverse()));

    // // Full reverse everything
    // OIConstants.operatorController.b().whileTrue(
    //     Commands.parallel(
    //         intake.reverse(),
    //         indexer.reverse(),
    //         shooter.reverse()));

    // // Override storing (flips it)
    // OIConstants.operatorController.x().whileTrue(indexer.overrideStoring());

    // // Handoff unjam
    // OIConstants.operatorController.y().whileTrue(
    //     Commands.parallel(
    //         intake.slowReverse(),
    //         indexer.forwards()));

    // Toggle pivot idle mode
    OIConstants.operatorController.start().whileTrue(
      Commands.startEnd(() -> pivot.toggleIdleMode(), () -> pivot.toggleIdleMode())
        .ignoringDisable(true));

    new Trigger(() -> Math.abs(OIConstants.operatorController.getLeftY()) > 0.15)
      .whileTrue(
        new RainbowCommand(() -> MathUtil.applyDeadband(OIConstants.operatorController.getLeftY(), 0.15)));

    if (Constants.currentMode == Mode.REAL){
      new Trigger(indexer::isStoring).onTrue(leds.intook().withTimeout(1));
      new Trigger(intake::isIntaking).debounce(0.1).whileTrue(leds.intaking());
    }

    timeRumble.onTrue(rumbleCommand(0.3).withTimeout(0.5));
    continuousRumble.whileTrue(rumbleCommand(0.6));
  }

  private void setBothRumble(double amount) {
    OIConstants.driverController.getHID().setRumble(RumbleType.kBothRumble, amount);
    OIConstants.operatorController.getHID().setRumble(RumbleType.kBothRumble, amount);
  }

  public Command rumbleCommand(double amount) {
    return Commands.startEnd(() -> setBothRumble(amount), () -> setBothRumble(0));
  }

  public void teleopInit() {
    shooter.setShootingSpeedRPM(0, 0);
    indexer.setIndexerSpeed(0);
    intake.setIntakeSpeed(0);
    pivot.reset();
  }

  private boolean between(double value, double min, double max){
    return min <= value && value <= max;
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