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
import frc.robot.Constants.CurrentLimits;
import frc.robot.util.KrakenLogger;

public class ShooterIOReal implements ShooterIO {
    private TalonFX topShooterMotor = new TalonFX(ShooterConstants.rightCanID);
    private TalonFX bottomShooterMotor = new TalonFX(ShooterConstants.leftCanID);

    // private TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    private boolean isEnabled;
    private boolean hasPlayed = false;

    // KrakenLogger topMotorLogger = new KrakenLogger(topShooterMotor, "Top shooter - ID " + ShooterConstants.rightCanID);
    // KrakenLogger bottomMotorLogger = new KrakenLogger(bottomShooterMotor, "Bottom shooter - ID " + ShooterConstants.leftCanID);

    private final StatusSignal<Double> topRPM = topShooterMotor.getVelocity();
    private final StatusSignal<Double> bottomRPM = bottomShooterMotor.getVelocity();
    private final StatusSignal<Double> topCurrent = bottomShooterMotor.getTorqueCurrent();
    private final StatusSignal<Double> bottomCurrent = bottomShooterMotor.getTorqueCurrent();

    private TalonFXConfiguration config(double kV) {
        var talonFXConfig = new TalonFXConfiguration();

        talonFXConfig.Slot0.kP = ShooterConstants.kP;
        talonFXConfig.Slot0.kI = 0;
        talonFXConfig.Slot0.kD = 0;
        talonFXConfig.Slot0.kS = 0;
        talonFXConfig.Slot0.kV = kV;

        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = CurrentLimits.shooter;

        talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        talonFXConfig.Audio.BeepOnBoot = true;

        return talonFXConfig;
    }

    public ShooterIOReal() {
        var topConfig = config(ShooterConstants.kVTop);
        var bottomConfig = config(ShooterConstants.kVBottom);

        BaseStatusSignal.setUpdateFrequencyForAll(50,
            topRPM,
            bottomRPM,
            topCurrent,
            bottomCurrent);

        topShooterMotor.optimizeBusUtilization(1.0);
        bottomShooterMotor.optimizeBusUtilization(1.0);

        bottomShooterMotor.clearStickyFaults();
        topShooterMotor.clearStickyFaults();

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = bottomShooterMotor.getConfigurator().apply(bottomConfig);
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
            status = topShooterMotor.getConfigurator().apply(topConfig);
            if (status.isOK()) break;
        }

        if (!status.isOK()) {
            System.out.println(
                    "Talon ID "
                            + topShooterMotor.getDeviceID()
                            + " failed config with error "
                            + status.toString());
        }
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
        BaseStatusSignal.refreshAll(topRPM, bottomRPM, topCurrent, bottomCurrent);

        // inputs.bottomShooterMotorAppliedVolts = bottomShooterMotor.getMotorVoltage().getValueAsDouble();
        inputs.bottomShooterMotorVelocityRPM = bottomRPM.getValueAsDouble() * 60.0; // RPS to RPM
        inputs.bottomShooterCurrent = bottomCurrent.getValueAsDouble();
        // inputs.bottomShooterAppliedVolts = bottomShooterMotor.getMotorVoltage().getValueAsDouble();

        // inputs.topShooterMotorAppliedVolts = topShooterMotor.getMotorVoltage().getValueAsDouble();
        inputs.topShooterMotorVelocityRPM = topRPM.getValueAsDouble() * 60.0; // RPS to RPM
        inputs.topShooterCurrent = topCurrent.getValueAsDouble();
        // inputs.topShooterAppliedVolts = topShooterMotor.getMotorVoltage().getValueAsDouble();

        isEnabled = DriverStation.isEnabled();


    }
} 