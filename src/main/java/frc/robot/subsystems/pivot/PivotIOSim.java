package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
// import frc.robot.util.SpringArmSim;

public class PivotIOSim implements PivotIO {
    private SingleJointedArmSim pivotSim = new SingleJointedArmSim(
        DCMotor.getNeoVortex(2), 
        PivotConstants.gearing, 
        1.9,
        0.39,
        Units.degreesToRadians(0), 
        Units.degreesToRadians(75), 
        false,
        Units.degreesToRadians(5));

    private double pivotAppliedVolts = 0.0;

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotPositionRads = pivotSim.getAngleRads();
        inputs.pivotMotorPositionRads = pivotSim.getAngleRads();
        inputs.pivotMotorVelocityRadPerSec = pivotSim.getVelocityRadPerSec();
        inputs.pivotCurrent = pivotSim.getCurrentDrawAmps();
        inputs.pivotAppliedVolts = pivotAppliedVolts;
        pivotSim.update(PivotConstants.kDt);
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
        pivotSim.setInputVoltage(pivotAppliedVolts - (-0.14 * pivotSim.getAngleRads() - 0.03));
    }
}
