package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.canID);
    
    private TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
    
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    public IntakeIOReal() {
        intakeMotor.setInverted(false);

        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = 80;
        // talonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        // talonFXConfig.CurrentLimits.SupplyCurrentLimit = 60;

        talonFXConfig.Audio.BeepOnBoot = true;
        
        talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        intakeMotor.clearStickyFaults();

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
        inputs.angularVelocityRPM = intakeMotor.getVelocity().getValueAsDouble()*60;
        inputs.angularPositionRot = intakeMotor.getPosition().getValueAsDouble();
        inputs.voltage = intakeMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setMotorSpeed(double speed) {
        intakeMotor.set(speed);
    }

}
