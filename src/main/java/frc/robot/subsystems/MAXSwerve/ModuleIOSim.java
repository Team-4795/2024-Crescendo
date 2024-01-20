package frc.robot.subsystems.MAXSwerve;

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

    private Rotation2d chassisAngularOffset;

    public ModuleIOSim(double chassisAngularOffset) {
        this.chassisAngularOffset = Rotation2d.fromRadians(chassisAngularOffset);
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

        driveSim.setState(driveSim.getAngularPositionRad(), optimizedDesiredState.speedMetersPerSecond / ModuleConstants.kDrivingEncoderVelocityFactor);
        turnSim.setState(state.angle.getRadians(), 0.0);
        optimizedState = new SwerveModuleState(optimizedDesiredState.speedMetersPerSecond, optimizedDesiredState.angle.minus(chassisAngularOffset));
    }

    

    @Override
    public SwerveModuleState getOptimizedState() {
        return optimizedState;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        driveSim.update(LOOP_PERIOD_SECS);
        turnSim.update(LOOP_PERIOD_SECS);

        inputs.drivePositionRad = driveSim.getAngularPositionRad() * ModuleConstants.kDrivingEncoderPositionFactor;
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec() * ModuleConstants.kDrivingEncoderVelocityFactor;
        inputs.turnAbsolutePosition = Rotation2d.fromRadians(turnSim.getAngularPositionRad()).minus(chassisAngularOffset);
        inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    }

}
