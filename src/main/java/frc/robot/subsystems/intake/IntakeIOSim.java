package frc.robot.subsystems.intake;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim frontroller = new DCMotorSim(DCMotor.getNEO(1), 30, 0.003);
  private final DCMotorSim backroller = new DCMotorSim(DCMotor.getNEO(1), 30, 0.003);


  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    
    frontroller.update(0.02);

    inputs.velocity = frontroller.getAngularVelocityRadPerSec();
    inputs.angularpositionrad = frontroller.getAngularPositionRad();
    inputs.angularpositionrot = frontroller.getAngularPositionRotations();
  	inputs.angularvelocityrad = frontroller.getAngularVelocityRadPerSec();
    inputs.angularvelocityRPM = frontroller.getAngularVelocityRPM();
    inputs.amps = frontroller.getCurrentDrawAmps();
  }

  @Override
  public void setMotorSpeed(double speed) {
    frontroller.setInputVoltage(speed);
  }
}

