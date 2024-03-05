package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO{
    DCMotorSim topMotor = new DCMotorSim(DCMotor.getKrakenX60(1), 1, 0.001);
    DCMotorSim bottomMotor = new DCMotorSim(DCMotor.getKrakenX60(1), 1, 0.001);

    SimpleMotorFeedforward ffd = new SimpleMotorFeedforward(0, ShooterConstants.kVTop / 60);
    PIDController controller = new PIDController(ShooterConstants.kP / 60, 0, 0);

    private double topSpeed = 0.0;
    private double bottomSpeed = 0.0;

    private double topAppliedVolts = 0.0;
    private double bottomAppliedVolts = 0.0;

    @Override
    public void runShooterMotorsRPM(double topSpeed, double bottomSpeed) {
        this.topSpeed = topSpeed;
        this.bottomSpeed = bottomSpeed;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        topMotor.update(0.02);
        bottomMotor.update(0.02);

        inputs.topShooterMotorAppliedVolts = topAppliedVolts;
        inputs.topShooterMotorVelocityRPM = topMotor.getAngularVelocityRPM();
        inputs.topShooterCurrent = topMotor.getCurrentDrawAmps();
        inputs.topShooterAppliedVolts = topAppliedVolts;

        inputs.bottomShooterMotorAppliedVolts = bottomAppliedVolts;
        inputs.bottomShooterMotorVelocityRPM = bottomMotor.getAngularVelocityRPM();
        inputs.bottomShooterCurrent = bottomMotor.getCurrentDrawAmps();
        inputs.bottomShooterAppliedVolts = bottomAppliedVolts;

        topAppliedVolts = MathUtil.clamp(
            ffd.calculate(topSpeed) + controller.calculate(topMotor.getAngularVelocityRPM(), topSpeed),
            -12, 12);
        bottomAppliedVolts = MathUtil.clamp(
            ffd.calculate(bottomSpeed) + controller.calculate(bottomMotor.getAngularVelocityRPM(), bottomSpeed), 
            -12, 12);

        topMotor.setInputVoltage(topAppliedVolts);
        bottomMotor.setInputVoltage(bottomAppliedVolts);
    }
}
