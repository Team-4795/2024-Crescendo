package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim implements PivotIO {
    private SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getNEO(2), 30, 1.79, 0.66, 0, 3 * Math.PI / 4, true, 0);

    private double pivotAppliedVolts = 0.0;

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        pivotSim.setInputVoltage(pivotAppliedVolts);
        pivotSim.update(PivotConstants.kDt);
        inputs.pivotRelativePosition = pivotSim.getAngleRads();
        inputs.pivotVelocityRadPerSec = pivotSim.getVelocityRadPerSec();
        inputs.pivotAppliedVolts = pivotAppliedVolts;
    }
    
    @Override
    public void rotatePivot(double speed) {
        pivotAppliedVolts = MathUtil.clamp(12 * speed, -12, 12);
        pivotSim.setInputVoltage(pivotAppliedVolts);
    }

}
