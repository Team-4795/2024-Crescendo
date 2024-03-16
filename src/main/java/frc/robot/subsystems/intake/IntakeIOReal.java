package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CurrentLimits;
import frc.robot.util.KrakenLogger;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.canID);
    
    private TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    KrakenLogger intakeLogger = new KrakenLogger(intakeMotor, "Intake - ID " + IntakeConstants.canID);

    private final StatusSignal<Double> current = intakeMotor.getStatorCurrent();
    private final StatusSignal<Double> voltage = intakeMotor.getMotorVoltage();
    private final StatusSignal<Double> velocity = intakeMotor.getVelocity();

    public IntakeIOReal() {
        // intakeMotor.setInverted(true);

        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = CurrentLimits.intakeKraken;

        // talonFXConfig.

        talonFXConfig.Audio.BeepOnBoot = true;
        
        talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        intakeMotor.clearStickyFaults();

        BaseStatusSignal.setUpdateFrequencyForAll(50, velocity, voltage, current);

        intakeMotor.optimizeBusUtilization(1.0);

        StatusCode response = intakeMotor.getConfigurator().apply(talonFXConfig);
        if (!response.isOK()) {
            System.out.println(
                    "Talon ID "
                            + intakeMotor.getDeviceID()
                            + " failed config with error "
                            + response.toString());
        }
    }

    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(velocity, voltage, current);
        
        inputs.angularVelocityRPM = velocity.getValueAsDouble() * 60;
        // inputs.angularPositionRot = intakeMotor.getPosition().getValueAsDouble();
        inputs.currentAmps = current.getValueAsDouble();
        inputs.voltage = voltage.getValueAsDouble();
    }

    @Override
    public void setMotorSpeed(double speed) {
        intakeMotor.set(speed);
    }

}
