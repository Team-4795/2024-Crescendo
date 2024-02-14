package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SpringArmSim;

public class PivotIOSim implements PivotIO {
    private SpringArmSim pivotSim = new SpringArmSim(
        DCMotor.getNeoVortex(2), 
        PivotConstants.gearing, 
        PivotConstants.inertia,
        PivotConstants.R,
        PivotConstants.M,
        PivotConstants.x,
        PivotConstants.y,
        PivotConstants.d,
        PivotConstants.F,
        Units.degreesToRadians(20), 
        Units.degreesToRadians(100), 
        Units.degreesToRadians(20));

    private double pivotAppliedVolts = 0.0;

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        pivotSim.update(PivotConstants.kDt);
        inputs.pivotPositionRads = pivotSim.getAngleRads() - PivotConstants.angleOffset;
        inputs.pivotMotorPositionRads = pivotSim.getAngleRads() - PivotConstants.angleOffset;
        inputs.pivotMotorVelocityRadPerSec = pivotSim.getVelocityRadPerSec();
        inputs.pivotAppliedVolts = pivotAppliedVolts;
    }
    
    @Override
    public void rotatePivot(double speed) {
        pivotAppliedVolts = MathUtil.clamp(12 * speed, -12, 12);
        pivotSim.setInputVoltage(pivotAppliedVolts);
    }

    @Override
    public void setVoltage(double volts){
        pivotAppliedVolts = volts;
        Logger.recordOutput("Setting output", volts);
        pivotSim.setInputVoltage(pivotAppliedVolts);
    }
}
