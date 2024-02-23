package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
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

    // KrakenLogger topMotorLogger = new KrakenLogger(topShooterMotor, "Top shooter - ID " + ShooterConstants.rightCanID);
    // KrakenLogger bottomMotorLogger = new KrakenLogger(bottomShooterMotor, "Bottom shooter - ID " + ShooterConstants.leftCanID);

    private final StatusSignal<Double> topRPM = topShooterMotor.getVelocity();
    private final StatusSignal<Double> bottomRPM = bottomShooterMotor.getVelocity();

    public ShooterIOReal() {
        talonFXConfig.Slot0.kP = ShooterConstants.kP;
        talonFXConfig.Slot0.kI = 0;
        talonFXConfig.Slot0.kD = 0;
        talonFXConfig.Slot0.kS = 0;
        talonFXConfig.Slot0.kV = ShooterConstants.kV;

        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = 80;

        talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        talonFXConfig.Audio.BeepOnBoot = true;

        BaseStatusSignal.setUpdateFrequencyForAll(50,
            topRPM,
            bottomRPM);

        topShooterMotor.optimizeBusUtilization(1.0);
        bottomShooterMotor.optimizeBusUtilization(1.0);

        bottomShooterMotor.clearStickyFaults();
        topShooterMotor.clearStickyFaults();

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = bottomShooterMotor.getConfigurator().apply(talonFXConfig);
            if (status.isOK()) break;
        }

        if (!status.isOK()) {
            System.out.println(
                    "Talon ID "
                            + bottomShooterMotor.getDeviceID()
                            + " failed config with error "
                            + status.toString());
        }

        status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = topShooterMotor.getConfigurator().apply(talonFXConfig);
            if (status.isOK()) break;
        }

        if (!status.isOK()) {
            System.out.println(
                    "Talon ID "
                            + topShooterMotor.getDeviceID()
                            + " failed config with error "
                            + status.toString());
        }

        hasPlayed = false;

        m_orchestra.addInstrument(bottomShooterMotor);
        m_orchestra.addInstrument(topShooterMotor);

        m_orchestra.loadMusic("chirp1Up.chrp");
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
        BaseStatusSignal.refreshAll(topRPM, bottomRPM);

        // inputs.bottomShooterMotorAppliedVolts = bottomShooterMotor.getMotorVoltage().getValueAsDouble();
        inputs.bottomShooterMotorVelocityRPM = bottomRPM.getValueAsDouble() * 60.0; // RPS to RPM
        // inputs.bottomShooterCurrent = bottomShooterMotor.getStatorCurrent().getValueAsDouble();
        // inputs.bottomShooterAppliedVolts = bottomShooterMotor.getMotorVoltage().getValueAsDouble();

        // inputs.topShooterMotorAppliedVolts = topShooterMotor.getMotorVoltage().getValueAsDouble();
        inputs.topShooterMotorVelocityRPM = topRPM.getValueAsDouble() * 60.0; // RPS to RPM
        // inputs.topShooterCurrent = topShooterMotor.getStatorCurrent().getValueAsDouble();
        // inputs.topShooterAppliedVolts = topShooterMotor.getMotorVoltage().getValueAsDouble();

        isEnabled = DriverStation.isEnabled();

        if (isEnabled && !hasPlayed && !DriverStation.isFMSAttached()) {
            m_orchestra.play();
            hasPlayed = true;
        } 

        if (hasPlayed && m_orchestra.isPlaying() && m_orchestra.getCurrentTime() > 1) {
            m_orchestra.stop();
        }
    }
} 