package frc.robot.subsystems.intake;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim frontroller = new DCMotorSim(DCMotor.getNEO(1), 30, 0.003);


  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    
    frontroller.update(0.02);

    inputs.angularPositionRot = frontroller.getAngularPositionRotations();
    inputs.angularVelocityRPM = frontroller.getAngularVelocityRPM();
    inputs.currentAmps = frontroller.getCurrentDrawAmps();
  }

  @Override
  public void setMotorSpeed(double speed) {
    frontroller.setInputVoltage(MathUtil.clamp(12 * speed, -12, 12));
  }
}

