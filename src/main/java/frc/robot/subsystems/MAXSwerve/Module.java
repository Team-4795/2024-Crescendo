// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MAXSwerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Module {
  private ModuleIO moduleIO;
  private ModuleIOInputsAutoLogged inputs;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public Module(ModuleIO io) {
    moduleIO = io;
    this.updateInputs();
  }

  public void updateInputs(){
    moduleIO.updateInputs(inputs);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(inputs.driveVelocityRadPerSec, inputs.turnAbsolutePosition);
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(inputs.drivePositionRad, inputs.turnAbsolutePosition);
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
