package frc.robot.subsystems.MAXSwerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.MAXSwerve.DriveConstants.ModuleConstants;

public class ModuleIOSim implements ModuleIO {
    public static final double DRIVE_GEAR_RATIO = (45.0 * 22.0) / (14.0 * 15.0);
    public static final double TURN_GEAR_RATIO = 9424.0 / 203.0;
    private static final double LOOP_PERIOD_SECS = 0.02;

    private DCMotorSim driveSim = new DCMotorSim(DCMotor.getNeoVortex(1), DRIVE_GEAR_RATIO, 0.02);
    private DCMotorSim turnSim = new DCMotorSim(DCMotor.getNeo550(1), TURN_GEAR_RATIO, 0.003);

    private SwerveModuleState optimizedState = new SwerveModuleState();

    private PIDController turnController = new PIDController(3, 0, 0);
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 2.002); //2.002
    private PIDController driveController = new PIDController(0.1, 0, 0);

    public ModuleIOSim(double chassisAngularOffset) {
        turnController.enableContinuousInput(0, 2 * Math.PI);
    }

    @Override
    public void resetEncoders() {
        driveSim.setState(0, 0);
        turnSim.setState(0, 0);
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(state,
                new Rotation2d(turnSim.getAngularPositionRad()));

        double driveVolts = feedforward.calculate(optimizedDesiredState.speedMetersPerSecond) + driveController.calculate(driveSim.getAngularVelocityRadPerSec() * ModuleConstants.kWheelDiameterMeters / 2, optimizedDesiredState.speedMetersPerSecond);
        double turnOutput = turnController.calculate(turnSim.getAngularPositionRad(), optimizedDesiredState.angle.getRadians());
        
        if (DriverStation.isEnabled()) {
            driveSim.setInputVoltage(MathUtil.clamp(driveVolts, -12, 12));
            turnSim.setInputVoltage(MathUtil.clamp(turnOutput, -12, 12));
        }

        optimizedState = new SwerveModuleState(optimizedDesiredState.speedMetersPerSecond, optimizedDesiredState.angle);
    }

    public double turnWrapping(double angleRad){
        if(angleRad >= 0){
            return (angleRad % (2 * Math.PI));
        } else {
            angleRad = angleRad % (2 * Math.PI);
            return angleRad + (2 * Math.PI);
        }
    }

    @Override
    public SwerveModuleState getOptimizedState() {
        return optimizedState;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        driveSim.update(LOOP_PERIOD_SECS);
        turnSim.update(LOOP_PERIOD_SECS);

        if (DriverStation.isDisabled()) {
            driveSim.setInputVoltage(0);
            turnSim.setInputVoltage(0);
        }

        inputs.drivePositionMeters = driveSim.getAngularPositionRad() * ModuleConstants.kWheelDiameterMeters / 2;
        inputs.driveVelocityMetersPerSec = driveSim.getAngularVelocityRadPerSec() * ModuleConstants.kWheelDiameterMeters / 2;
        inputs.turnAbsolutePosition = Rotation2d.fromRadians(turnSim.getAngularPositionRad());
        inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    }

}
