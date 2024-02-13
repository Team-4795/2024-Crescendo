package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;

public class ShooterIOReal implements ShooterIO {
    
    private TalonFX topShooterMotor = new TalonFX(ShooterConstants.rightCanID);
    private TalonFX bottomShooterMotor = new TalonFX(ShooterConstants.leftCanID);

    private TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    public ShooterIOReal(){
        talonFXConfig.Slot0.kP = 0.05;
        talonFXConfig.Slot0.kI = 0;
        talonFXConfig.Slot0.kD = 0;
        talonFXConfig.Slot0.kS = 0;
        talonFXConfig.Slot0.kV = 0;

        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = 60;

        talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // talonFXConfig.MotionMagic.MotionMagicAcceleration = 100;
        // talonFXConfig.MotionMagic.MotionMagicCruiseVelocity = 10;

        talonFXConfig.Audio.BeepOnBoot = true;

        bottomShooterMotor.clearStickyFaults();
        topShooterMotor.clearStickyFaults();

        StatusCode response = bottomShooterMotor.getConfigurator().apply(talonFXConfig);
        if (!response.isOK()) {
            System.out.println(
                    "Talon ID "
                            + bottomShooterMotor.getDeviceID()
                            + " failed config with error "
                            + response.toString());
        }

        response = topShooterMotor.getConfigurator().apply(talonFXConfig);
        if (!response.isOK()) {
            System.out.println(
                    "Talon ID "
                            + topShooterMotor.getDeviceID()
                            + " failed config with error "
                            + response.toString());
        }
    }

    @Override
    public void runShooterMotors(double topSpeed, double bottomSpeed) {
        // set velocity to certain rps, add 0.5 V to overcome gravity
        topShooterMotor.setControl(m_request.withVelocity(topSpeed).withFeedForward(0.2));
        bottomShooterMotor.setControl(m_request.withVelocity(bottomSpeed).withFeedForward(0.2));

        // bottomShooterMotor.set(MathUtil.clamp(bottomSpeed, -1, 1));
        // topShooterMotor.set(MathUtil.clamp(topSpeed, -1, 1));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.bottomShooterMotorAppliedVolts = bottomShooterMotor.getMotorVoltage().getValueAsDouble();
        inputs.bottomShooterMotorVelocityRPM = bottomShooterMotor.getVelocity().getValueAsDouble() * 60.0; // RPS to RPM
        inputs.topShooterMotorAppliedVolts = topShooterMotor.getMotorVoltage().getValueAsDouble();
        inputs.topShooterMotorVelocityRPM = topShooterMotor.getVelocity().getValueAsDouble() * 60.0; // RPS to RPM

    }
}