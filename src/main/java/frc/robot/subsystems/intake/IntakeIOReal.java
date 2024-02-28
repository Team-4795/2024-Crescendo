package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.util.KrakenLogger;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.canID);
    
    private TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
    
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    // KrakenLogger intakeLogger = new KrakenLogger(intakeMotor, "Intake - ID " + IntakeConstants.canID);

    private final StatusSignal<Double> current = intakeMotor.getStatorCurrent();
    private final StatusSignal<Double> voltage = intakeMotor.getMotorVoltage();
    private final StatusSignal<Double> velocity = intakeMotor.getVelocity();

    public IntakeIOReal() {
        intakeMotor.setInverted(true);

        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = 80;
        // talonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        // talonFXConfig.CurrentLimits.SupplyCurrentLimit = 60;

        talonFXConfig.Audio.BeepOnBoot = true;
        
        talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        intakeMotor.clearStickyFaults();

        // intakeMotor.getVelocity().setUpdateFrequency(50);
        // intakeMotor.getMotorVoltage().setUpdateFrequency(50);
        // intakeMotor.getPosition().setUpdateFrequency(50);

        BaseStatusSignal.setUpdateFrequencyForAll(50, velocity, voltage);

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
        BaseStatusSignal.refreshAll(velocity, voltage);
        
        inputs.angularVelocityRPM = velocity.getValueAsDouble()*60;
        // inputs.angularPositionRot = intakeMotor.getPosition().getValueAsDouble();
        inputs.currentAmps = current.getValueAsDouble();
        inputs.voltage = voltage.getValueAsDouble();
    }

    @Override
    public void setMotorSpeed(double speed) {
        intakeMotor.set(speed);
    }

}
