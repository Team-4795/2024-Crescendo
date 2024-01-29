package frc.robot.subsystems.MAXSwerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.MAXSwerve.DriveConstants.ModuleConstants;

public class ModuleIOSim implements ModuleIO {
    public static final double DRIVE_GEAR_RATIO = (45.0 * 22.0) / (14.0 * 15.0);
    public static final double TURN_GEAR_RATIO = 9424.0 / 203.0;
    private static final double LOOP_PERIOD_SECS = 0.02;

    private DCMotorSim driveSim = new DCMotorSim(DCMotor.getNEO(1), DRIVE_GEAR_RATIO, 0.02);
    private DCMotorSim turnSim = new DCMotorSim(DCMotor.getNeo550(1), TURN_GEAR_RATIO, 0.003);

    private SwerveModuleState optimizedState = new SwerveModuleState();

    private PIDController turnController = new PIDController(1, 0, 0);

    private Rotation2d chassisAngularOffset;

    public ModuleIOSim(double chassisAngularOffset) {
        this.chassisAngularOffset = Rotation2d.fromRadians(chassisAngularOffset);
        turnController.enableContinuousInput(0, 2 * Math.PI);
    }

    @Override
    public void resetEncoders() {
        driveSim.setState(0, 0);
        turnSim.setState(0, 0);
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = state.speedMetersPerSecond;
        correctedDesiredState.angle = state.angle.plus(chassisAngularOffset);

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                new Rotation2d(turnSim.getAngularPositionRad()));

        double driveVolts = MathUtil.clamp(12 * optimizedDesiredState.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond, -12, 12);
        double turnOutput = turnController.calculate(turnWrapping(turnSim.getAngularPositionRad()), optimizedDesiredState.angle.getRadians());
        
        driveSim.setInputVoltage(driveVolts);
        turnSim.setInputVoltage(MathUtil.clamp(12 * turnOutput, -12, 12));
        optimizedState = new SwerveModuleState(optimizedDesiredState.speedMetersPerSecond, optimizedDesiredState.angle.minus(chassisAngularOffset));
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

        inputs.drivePositionMeters = (driveSim.getAngularPositionRad() * ModuleConstants.kWheelCircumferenceMeters / (2 * Math.PI));
        inputs.driveVelocityMetersPerSec = (driveSim.getAngularVelocityRadPerSec() * ModuleConstants.kWheelCircumferenceMeters / (2 * Math.PI));
        inputs.turnAbsolutePosition = Rotation2d.fromRadians(turnSim.getAngularPositionRad()).minus(chassisAngularOffset);
        inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    }

}
