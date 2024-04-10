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

import java.util.Map;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonTargetSortMode;

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
import frc.robot.autoPaths.GDA_AS1456;
import frc.robot.autoPaths.GDA_M2145;
import frc.robot.autoPaths.GDA_M2145_RunEverything;
import frc.robot.autoPaths.GDA_M32145;
import frc.robot.autoPaths.GDA_SS8765;
import frc.robot.subsystems.MAXSwerve.*;
import frc.robot.subsystems.Shooter.*;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.pivot.*;
import frc.robot.subsystems.vision.intakeCam.*;
import frc.robot.subsystems.vision.AprilTagVision.*;
import frc.robot.util.AutoGenerator;
import frc.robot.util.NamedCommandManager;
import frc.robot.util.NoteVisualizer;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.*;

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
  private final IntakeCamVision intakeCamVision;
  private final Shooter shooter;
  private final Pivot pivot;
  private final Indexer indexer;
  private final Intake intake;
  private LEDs leds;

  private AutoGenerator generator;
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
        vision = Vision.initialize(new VisionIOReal(0), new VisionIOReal(1), new VisionIOReal(2));
        // vision = Vision.initialize(new VisionIOSim());
        intakeCamVision = IntakeCamVision.initialize(new IntakeCamVisionIOReal());
        drive = Drive.initialize(
            new GyroIOPigeon2(),
            new ModuleIOSparkFlex(DriveConstants.kFrontLeftDrivingCanId, DriveConstants.kFrontLeftTurningCanId,
                DriveConstants.kFrontLeftChassisAngularOffset),
            new ModuleIOSparkFlex(DriveConstants.kFrontRightDrivingCanId, DriveConstants.kFrontRightTurningCanId,
                DriveConstants.kFrontRightChassisAngularOffset),
            new ModuleIOSparkFlex(DriveConstants.kRearLeftDrivingCanId, DriveConstants.kRearLeftTurningCanId,
                DriveConstants.kBackLeftChassisAngularOffset),
            new ModuleIOSparkFlex(DriveConstants.kRearRightDrivingCanId, DriveConstants.kRearRightTurningCanId,
                DriveConstants.kBackRightChassisAngularOffset));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        intake = Intake.initialize(new IntakeIOSim());
        shooter = Shooter.initialize(new ShooterIOSim());
        pivot = Pivot.initialize(new PivotIOSim());
        indexer = Indexer.initialize(new IndexerIOSim());
        vision = Vision.initialize(new VisionIOSim());
        intakeCamVision = IntakeCamVision.initialize(new IntakeCamVisionIO() {});
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
        intakeCamVision = IntakeCamVision.initialize(new IntakeCamVisionIO() {});
        break;
    }

    leds = LEDs.getInstance();

    NamedCommandManager.registerAll();
    NoteVisualizer.setPivotPoseSupplier(pivot::getPose);
    
    generator = new AutoGenerator();
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());


    // autoChooser.addOption("Pivot SysIs (Quasistatic Forward)", pivot.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption("Pivot SysIs (Quasistatic Reverse)", pivot.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption("Pivot SysIs (Dynamic Forward)", pivot.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption("Pivot SysIs (Dynamic everse)", pivot.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption("Pivot Model", new ArmFeedForwardCharacterization(pivot, (volts) -> pivot.runVoltage(volts), () -> pivot.getVelocity(), () -> pivot.getPosition(), (x) -> 0.0));
    autoChooser.addOption("TEST - SS GP 8765", GDA_SS8765.load());
    autoChooser.addOption("TEST - AS GP 456", GDA_AS1456.load());
    autoChooser.addOption("Auto Generated Routine", null);
    // autoChooser.addOption("TEST - M GP 32145", GDA_M32145.load());


    // Configure the button bindings
    configureButtonBindings();

  }

  /* Returns if the robot is ready to outtake */
  public boolean readyToShoot() {
    return pivot.atGoal() && shooter.atGoal() && (AlignPose.atGoal() || !StateManager.isAutomate()) && (drive.slowMoving() || StateManager.getState() == StateManager.State.SHUTTLE);
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
    Trigger timeRumble = new Trigger(() -> between(DriverStation.getMatchTime(), 19.5, 20.5) || between(DriverStation.getMatchTime(), 39.5, 40.5));
    Trigger continuousRumble = new Trigger(() -> DriverStation.getMatchTime() <= 5);
    
    Trigger isReady = new Trigger(this::readyToShoot);
    Trigger isReadyRumble = isReady.and(StateManager::isAiming);

    // Gamepiece align
    OIConstants.driverController.rightBumper().whileTrue(new AlignToGamepiece());

    // Align Amp / Speaker
    OIConstants.driverController.leftBumper().whileTrue(
      Commands.either(
        Commands.select(
          Map.ofEntries(
            Map.entry(State.AMP, drive.AutoAlignAmp()),
            Map.entry(State.SPEAKER, Commands.parallel(
              new AlignSpeaker(),
              pivot.aimSpeakerDynamic(),
              shooter.revSpeaker()
            )),
            Map.entry(State.SHUTTLE, new AlignShuttle())),
            StateManager::getState),
        shooter.revSpeaker()
          .alongWith(pivot.aimSpeakerDynamic()),
        () -> StateManager.isAutomate()
      )
    );

    // Auto Shoot
    OIConstants.driverController.rightTrigger(0.3)
      .and(isReady)
      .whileTrue(indexer.forwards().finallyDo(() -> StateManager.setState(State.SPEAKER)))
      .onTrue(NoteVisualizer.shoot());
    
    //Drive robot relative
    // OIConstants.driverController.leftTrigger(0.3)
    //   .onTrue(Commands.runOnce(() -> drive.setFieldRelative(false)))
    //   .onFalse(Commands.runOnce(() -> drive.setFieldRelative(true)));

    // Normal Shoot (Might switch onto different button as a toggle mode)
    OIConstants.driverController.leftTrigger(0.3)
      .whileTrue(indexer.forwards().finallyDo(() -> StateManager.setState(State.SPEAKER)))
      .onTrue(NoteVisualizer.shoot());

    // SADDEST BUTTON IN EXISTENCE ON PERSEUS
    OIConstants.driverController.x().onTrue(Commands.runOnce(() -> {
      StateManager.toggleAutomate();
      leds.toggleYellow();
    }));

    // Zero heading
    OIConstants.driverController.a().whileTrue(Commands.runOnce(drive::zeroHeading));

    // Non auto amp align
    OIConstants.operatorController.povLeft().whileTrue(pivot.aimAmp().alongWith(shooter.revAmp()));
    
    // Speaker mode
    OIConstants.operatorController.leftBumper()
      .and(OIConstants.operatorController.rightBumper().negate())
      .onTrue(Commands.runOnce(() -> StateManager.setState(State.SPEAKER)));

    // Amp mode
    OIConstants.operatorController.rightBumper()
      .and(OIConstants.operatorController.leftBumper().negate())
      .onTrue(Commands.runOnce(() -> StateManager.setState(State.AMP)));

    // Shuttle mode
    OIConstants.operatorController.povRight()
      .whileTrue((Commands.startEnd(
        () -> StateManager.setState(State.SHUTTLE),
        () -> StateManager.setState(State.SPEAKER))));

    // Source Intake
    OIConstants.operatorController.povUp().onTrue(
        Commands.parallel(
            shooter.slowReverse(),
            indexer.slowReverse(),
            pivot.aimSource()));
    
    // Ground Intake
    OIConstants.operatorController.povDown().or(OIConstants.operatorController.povDownLeft()).or(OIConstants.operatorController.povDownRight())
    .whileTrue(
        Commands.parallel(
            pivot.aimIntake(),
            intake.intake(),
            indexer.forwards())
        .until(indexer::isStoring)
        .andThen(
          indexer.reverse().withTimeout(0.05)
        ).onlyIf(() -> !indexer.isStoring())
    );

    // Slow reverse tower
    OIConstants.operatorController.a().whileTrue(
            Commands.parallel(
                indexer.slowReverse(),
                shooter.slowReverse()));

    // Full reverse everything
    OIConstants.operatorController.b().whileTrue(
        Commands.parallel(
            intake.reverse(),
            indexer.reverse(),
            shooter.reverse()));

    // Override storing (flips it)
    OIConstants.operatorController.x().whileTrue(indexer.overrideStoring().ignoringDisable(true));

    // Handoff unjam
    OIConstants.operatorController.y().whileTrue(
        Commands.parallel(
            intake.slowReverse(),
            indexer.forwards()));

    // Toggle pivot idle mode
    OIConstants.operatorController.start().whileTrue(
      Commands.startEnd(() -> pivot.toggleIdleMode(), () -> pivot.toggleIdleMode())
        .ignoringDisable(true));

    new Trigger(() -> Math.abs(OIConstants.operatorController.getLeftY()) > 0.15)
      .whileTrue(
        new RainbowCommand(() -> MathUtil.applyDeadband(OIConstants.operatorController.getLeftY(), 0.15)));

    if (Constants.currentMode == Mode.REAL){
      new Trigger(indexer::isStoring).onTrue(rumbleCommand(0.6).withTimeout(0.75));
      new Trigger(intake::isIntaking).debounce(0.1).whileTrue(Commands.parallel(
        new RainbowCommand(() -> 0.8),
        rumbleCommand(0.4)
      ));
    }

    timeRumble.onTrue(rumbleCommand(0.3).withTimeout(1));
    continuousRumble.whileTrue(rumbleCommand(0.4));
    isReadyRumble.whileTrue(rumbleCommand(0.6));
  }

  private void setBothRumble(double amount) {
    OIConstants.driverController.getHID().setRumble(RumbleType.kBothRumble, amount);
    OIConstants.operatorController.getHID().setRumble(RumbleType.kBothRumble, amount);
  }

  public Command rumbleCommand(double amount) {
    return Commands.startEnd(() -> setBothRumble(amount), () -> setBothRumble(0));
  }

  public void init() {
    intakeCamVision.setTargetComparator(PhotonTargetSortMode.Largest);
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
    if(autoChooser.get() != null){
      return autoChooser.get();
    } else {
      return generator.getGeneratedAuto();
    }
  }
}