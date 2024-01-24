package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim implements PivotIO {
    private SingleJointedArmSim pivotSim;

    private double pivotAppliedVolts = 0.0;

    public PivotIOSim() {
        pivotSim = new SingleJointedArmSim(DCMotor.getNeoVortex(2), 30, 1.5, 0.66, 0,90 , true, 0);
    }

    public void updateInputs(PivotIOInputs inputs) {
        pivotSim.update(0.02);

        inputs.pivotRelativePosition = pivotSim.getAngleRads() / (2* Math.PI);
        inputs.pivotVelocityRadPerSec = pivotSim.getVelocityRadPerSec () / (2*Math.PI);
        inputs.pivotAppliedVolts = pivotAppliedVolts;

    }
    
    public void setARmVoltage(double volts) {
        pivotAppliedVolts = MathUtil.clamp(volts, -12, 12);
        pivotSim.setInputVoltage(pivotAppliedVolts);

    }



}
