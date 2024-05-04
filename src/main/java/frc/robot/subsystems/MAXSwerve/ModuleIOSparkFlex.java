// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MAXSwerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.CurrentLimits;
import frc.robot.subsystems.MAXSwerve.DriveConstants.ModuleConstants;

public class ModuleIOSparkFlex implements ModuleIO {
  private final CANSparkFlex m_drivingSpark;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkPIDController m_drivingPIDController;
  private final SparkPIDController m_turningPIDController;

  private Rotation2d m_chassisAngularOffset;

  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  private SwerveModuleState optimizedState = new SwerveModuleState(0.0, new Rotation2d());

  public ModuleIOSparkFlex(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSpark = new CANSparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingSpark.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_drivingPIDController = m_drivingSpark.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();

    m_drivingSpark.setCANTimeout(250);
    m_turningSparkMax.setCANTimeout(250);

    m_drivingSpark.enableVoltageCompensation(12);
    m_turningSparkMax.enableVoltageCompensation(12);

    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    m_turningEncoder.setAverageDepth(2);
    m_drivingEncoder.setAverageDepth(2);

    m_drivingEncoder.setMeasurementPeriod(20);

    // in meters and meters per second
    m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

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
    m_drivingPIDController.setP(ModuleConstants.kDrivingP);
    m_drivingPIDController.setI(ModuleConstants.kDrivingI);
    m_drivingPIDController.setD(ModuleConstants.kDrivingD);
    m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
    m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);
    // Set the PID gains for the turning motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(ModuleConstants.kTurningP);
    m_turningPIDController.setI(ModuleConstants.kTurningI);
    m_turningPIDController.setD(ModuleConstants.kTurningD);
    m_turningPIDController.setFF(ModuleConstants.kTurningFF);
    m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    m_drivingSpark.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    m_drivingSpark.setSmartCurrentLimit(CurrentLimits.drive);
    m_turningSparkMax.setSmartCurrentLimit(CurrentLimits.turning);

    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 1000);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

    m_drivingSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    m_drivingSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    m_drivingSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    m_drivingSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
    m_drivingSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSpark.burnFlash();
    m_turningSparkMax.burnFlash();

    m_drivingSpark.setCANTimeout(0);
    m_turningSparkMax.setCANTimeout(0);

    m_chassisAngularOffset = Rotation2d.fromRadians(chassisAngularOffset);
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  @Override
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
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
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = state;
  }

  @Override
  public SwerveModuleState getOptimizedState() {
    return optimizedState;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionMeters = m_drivingEncoder.getPosition();
    inputs.driveVelocityMetersPerSec = m_drivingEncoder.getVelocity();

    inputs.turnAbsolutePosition = Rotation2d.fromRadians(m_turningEncoder.getPosition()).minus(m_chassisAngularOffset);
    inputs.turnVelocityRadPerSec = m_turningEncoder.getVelocity();
  }

}