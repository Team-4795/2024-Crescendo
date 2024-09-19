// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MAXSwerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.CurrentLimits;
import frc.robot.subsystems.MAXSwerve.DriveConstants.ModuleConstants;

public class ModuleIOTalon implements ModuleIO {
  private final TalonFX m_drivingTalon;
  private final CANSparkMax m_turningSparkMax;

  private final AbsoluteEncoder m_turningEncoder;

  private final SparkPIDController m_turningPIDController;

  private Rotation2d m_chassisAngularOffset;

  final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  private SwerveModuleState optimizedState = new SwerveModuleState(0.0, new Rotation2d());

  public ModuleIOTalon(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingTalon = new TalonFX(drivingCANId);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    // m_drivingTalon.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_turningPIDController = m_turningSparkMax.getPIDController();

    // m_drivingTalon.setCANTimeout(250);
    m_turningSparkMax.setCANTimeout(250);

    // m_drivingTalon.enableVoltageCompensation(12);
    m_turningSparkMax.enableVoltageCompensation(12);

    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    m_turningEncoder.setAverageDepth(2);

    m_drivingTalon.optimizeBusUtilization();

    // in radians and radians per second
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!

    // Set the PID gains for the turning motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(ModuleConstants.kTurningP);
    m_turningPIDController.setI(ModuleConstants.kTurningI);
    m_turningPIDController.setD(ModuleConstants.kTurningD);
    m_turningPIDController.setFF(ModuleConstants.kTurningFF);
    m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    m_turningSparkMax.setSmartCurrentLimit(CurrentLimits.turning);

    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 1000);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

    // m_drivingTalon.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    // m_drivingTalon.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    // m_drivingTalon.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    // m_drivingTalon.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
    // m_drivingTalon.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_turningSparkMax.burnFlash();

    // m_drivingTalon.setCANTimeout(0);
    m_turningSparkMax.setCANTimeout(0);

    m_chassisAngularOffset = Rotation2d.fromRadians(chassisAngularOffset);
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingTalon.setPosition(0);

    var config = config();

    m_drivingTalon.optimizeBusUtilization(1.0);

    m_drivingTalon.clearStickyFaults();

    StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = m_drivingTalon.getConfigurator().apply(config);
            if (status.isOK()) break;
        }

        if (!status.isOK()) {
            System.out.println(
                    "Talon ID "
                            + m_drivingTalon.getDeviceID()
                            + " failed config with error "
                            + status.toString());
        }
  }

  private TalonFXConfiguration config() {
        var talonFXConfig = new TalonFXConfiguration();

        talonFXConfig.Slot0.kP = ModuleConstants.kDrivingP;
        talonFXConfig.Slot0.kI = ModuleConstants.kDrivingI;
        talonFXConfig.Slot0.kD = ModuleConstants.kDrivingD;
        talonFXConfig.Slot0.kS = 0;
        talonFXConfig.Slot0.kV = ModuleConstants.kDrivingFF;

        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = CurrentLimits.drive;

        talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        talonFXConfig.Audio.BeepOnBoot = true;

        return talonFXConfig;
    }

  @Override
  public void resetEncoders() {
    m_drivingTalon.setPosition(0);
  }

  @Override
  public void setDesiredState(SwerveModuleState state) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = state.speedMetersPerSecond;
    correctedDesiredState.angle = state.angle.plus(m_chassisAngularOffset);

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    optimizedState = optimizedDesiredState;
    // optimizedState.angle = optimizedState.angle.minus(m_chassisAngularOffset);

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingTalon.setControl(m_request.withVelocity(optimizedDesiredState.speedMetersPerSecond / ModuleConstants.kWheelCircumferenceMeters));
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = state;
  }

  @Override
  public SwerveModuleState getOptimizedState() {
    return optimizedState;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionMeters = m_drivingTalon.getPosition().getValueAsDouble() * ModuleConstants.kWheelCircumferenceMeters;
    inputs.driveVelocityMetersPerSec = m_drivingTalon.getVelocity().getValueAsDouble() * ModuleConstants.kWheelCircumferenceMeters;

    inputs.turnAbsolutePosition = Rotation2d.fromRotations(m_drivingTalon.getPosition().getValueAsDouble()).minus(m_chassisAngularOffset);
    inputs.turnVelocityRadPerSec = m_drivingTalon.getVelocity().getValueAsDouble() * 2 * Math.PI;
  }

}
