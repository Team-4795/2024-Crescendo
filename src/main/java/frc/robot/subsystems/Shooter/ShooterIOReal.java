package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.KrakenLogger;

public class ShooterIOReal implements ShooterIO {
    private TalonFX topShooterMotor = new TalonFX(ShooterConstants.rightCanID);
    private TalonFX bottomShooterMotor = new TalonFX(ShooterConstants.leftCanID);

    private Orchestra m_orchestra = new Orchestra();

    private TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    private boolean isEnabled;
    private boolean hasPlayed;

    KrakenLogger topMotorLogger = new KrakenLogger(topShooterMotor, "Top shooter - ID " + ShooterConstants.rightCanID);
    KrakenLogger bottomMotorLogger = new KrakenLogger(bottomShooterMotor, "Bottom shooter - ID " + ShooterConstants.leftCanID);

    public ShooterIOReal() {
        talonFXConfig.Slot0.kP = ShooterConstants.kP;
        talonFXConfig.Slot0.kI = 0;
        talonFXConfig.Slot0.kD = 0;
        talonFXConfig.Slot0.kS = 0;
        talonFXConfig.Slot0.kV = ShooterConstants.kV;

        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = 80;

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

        hasPlayed = false;

        m_orchestra.addInstrument(bottomShooterMotor);
        m_orchestra.addInstrument(topShooterMotor);

        m_orchestra.loadMusic("chirpWindowsXPFX.chrp");
    }

    @Override
    public void runVoltageTop(double volts) {
        topShooterMotor.setVoltage(volts);
    }

    @Override
    public void runVoltageBottom(double volts) {
        bottomShooterMotor.setVoltage(volts);
    }

    @Override
    public void runShooterMotorsRPM(double topSpeed, double bottomSpeed) {
        topShooterMotor.setControl(m_request.withVelocity(topSpeed / 60));
        bottomShooterMotor.setControl(m_request.withVelocity(bottomSpeed / 60));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.bottomShooterMotorAppliedVolts = bottomShooterMotor.getMotorVoltage().getValueAsDouble();
        inputs.bottomShooterMotorVelocityRPM = bottomShooterMotor.getVelocity().getValueAsDouble() * 60.0; // RPS to RPM
        inputs.bottomShooterCurrent = bottomShooterMotor.getStatorCurrent().getValueAsDouble();
        inputs.bottomShooterAppliedVolts = bottomShooterMotor.getMotorVoltage().getValueAsDouble();

        inputs.topShooterMotorAppliedVolts = topShooterMotor.getMotorVoltage().getValueAsDouble();
        inputs.topShooterMotorVelocityRPM = topShooterMotor.getVelocity().getValueAsDouble() * 60.0; // RPS to RPM
        inputs.topShooterCurrent = topShooterMotor.getStatorCurrent().getValueAsDouble();
        inputs.topShooterAppliedVolts = topShooterMotor.getMotorVoltage().getValueAsDouble();

        isEnabled = DriverStation.isEnabled();

        if (isEnabled && !hasPlayed && !DriverStation.isFMSAttached()) {
            m_orchestra.play();
            hasPlayed = true;
        }
    }
}