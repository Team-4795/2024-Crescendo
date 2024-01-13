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

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final AbsoluteEncoder turnAbsoluteEncoder;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMax(int index) {
    switch (index) {
      case 0:
        driveSparkMax = new CANSparkMax(8, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(9, MotorType.kBrushless);
        absoluteEncoderOffset = new Rotation2d(Math.PI); // MUST BE CALIBRATED
        break;
      case 1:
        driveSparkMax = new CANSparkMax(6, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(7, MotorType.kBrushless);
        absoluteEncoderOffset = new Rotation2d(Math.PI / 2); // MUST BE CALIBRATED
        break;
      case 2:
        driveSparkMax = new CANSparkMax(4, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(5, MotorType.kBrushless);
        absoluteEncoderOffset = new Rotation2d(-Math.PI / 2); // MUST BE CALIBRATED
        break;
      case 3:
        driveSparkMax = new CANSparkMax(2, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(3, MotorType.kBrushless);
        absoluteEncoderOffset = new Rotation2d(0); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    driveSparkMax.restoreFactoryDefaults();
    turnSparkMax.restoreFactoryDefaults();

    
    
    // driveSparkMax.setCANTimeout(250);
    // turnSparkMax.setCANTimeout(250);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();
    turnAbsoluteEncoder = turnSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

    turnSparkMax.setInverted(isTurnMotorInverted);
    driveSparkMax.setSmartCurrentLimit(60);
    turnSparkMax.setSmartCurrentLimit(25);
    // driveSparkMax.enableVoltageCompensation(12.0);
    // turnSparkMax.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    // driveEncoder.setMeasurementPeriod(10);
    // driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    // turnRelativeEncoder.setMeasurementPeriod(10);
    // turnRelativeEncoder.setAverageDepth(2);

    driveEncoder.setPositionConversionFactor(2.0 * Math.PI / Module.DRIVE_GEAR_RATIO); // Rev -> Rad
    driveEncoder.setVelocityConversionFactor(2.0 * Math.PI / Module.DRIVE_GEAR_RATIO / 60.0); // RPM -> Rad/s

    turnAbsoluteEncoder.setPositionConversionFactor(2.0 * Math.PI);
    turnAbsoluteEncoder.setVelocityConversionFactor(2.0 * Math.PI / 60.0);

    turnRelativeEncoder.setPositionConversionFactor(1.0 / Module.TURN_GEAR_RATIO); // Keep in revs
    turnRelativeEncoder.setVelocityConversionFactor(2.0 * Math.PI / Module.TURN_GEAR_RATIO / 60.0);

    // driveSparkMax.setCANTimeout(0);
    // turnSparkMax.setCANTimeout(0);

    // driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    // driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);

    driveSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    turnSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    drivePositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(driveEncoder::getPosition);
    turnPositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(turnRelativeEncoder::getPosition);

    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();
  }
  
  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = driveEncoder.getPosition();
    inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

    inputs.turnAbsolutePosition =
        new Rotation2d(turnAbsoluteEncoder.getPosition()).minus(absoluteEncoderOffset);
    inputs.turnPosition = Rotation2d.fromRotations(turnRelativeEncoder.getPosition());
    inputs.turnVelocityRadPerSec = turnRelativeEncoder.getVelocity();
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};

    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> value)
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
