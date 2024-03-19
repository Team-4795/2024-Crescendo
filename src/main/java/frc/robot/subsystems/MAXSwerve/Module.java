// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MAXSwerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Module {
  private ModuleIO moduleIO;
  private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private int index;

  private double lastPositionMeters = 0.0; // Used for delta calculation
  private SwerveModulePosition positionDelta;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public Module(ModuleIO io, int index) {
    moduleIO = io;
    this.updateInputs();
    this.index = index;
  }

  public void updateInputs(){
    moduleIO.updateInputs(inputs);
  }

  public void periodic() {
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    double positionMeters = inputs.drivePositionMeters;
    Rotation2d angleRad = inputs.turnAbsolutePosition; 
    positionDelta = new SwerveModulePosition(inputs.drivePositionMeters - lastPositionMeters, angleRad);
    lastPositionMeters = positionMeters;

    int sampleCount = inputs.drivePositions.length;
    odometryPositions = new SwerveModulePosition[sampleCount];
    for(int i = 0; i < sampleCount; i++){
      double position = inputs.drivePositions[i];
      Rotation2d angle = inputs.turnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(position, angle);
    }
  }

  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  public double[] getOdometryTimestamps() {
    return inputs.timeStamps;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(inputs.driveVelocityMetersPerSec, inputs.turnAbsolutePosition);
  }

  public SwerveModuleState getOptimizedState() {
    return moduleIO.getOptimizedState();
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(inputs.drivePositionMeters, inputs.turnAbsolutePosition);
  }

  public SwerveModulePosition getPositionDelta() {
    return positionDelta;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    moduleIO.setDesiredState(desiredState);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    moduleIO.resetEncoders();
  }
}
