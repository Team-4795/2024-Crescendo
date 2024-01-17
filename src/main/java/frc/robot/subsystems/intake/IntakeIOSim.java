package frc.robot.subsystems.intake;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim frontroller = new DCMotorSim(DCMotor.getNEO(1), 1, 0.003);
  private final DCMotorSim backroller = new DCMotorSim(DCMotor.getNEO(1), 1, 0.003);

  private boolean closedLoop = false;
  private double velocity = 0.0;
  private double voltage = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (closedLoop) {
      velocity =
          frontroller.getAngularVelocityRPM();
    }

    frontroller.update(0.02);

    inputs.velocity = frontroller.getAngularVelocityRadPerSec();
    //inputs.voltage = frontroller.getOutputCurrent();
  }

  @Override
  public void setMotorSpeed(double speed) {
    closedLoop = true;
    frontroller.setInputVoltage(30);
  }
}

