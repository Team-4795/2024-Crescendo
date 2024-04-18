package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import frc.robot.Constants.CurrentLimits;

public class IndexerIOReal implements IndexerIO {
    private CANSparkMax bottomIndexMotor = new CANSparkMax(IndexerConstants.leftCanID, MotorType.kBrushless);
    private CANSparkMax towerIndexMotor = new CANSparkMax(IndexerConstants.rightCanID, MotorType.kBrushless);
    private RelativeEncoder bottomEncoder = bottomIndexMotor.getEncoder();
    private RelativeEncoder towerEncoder = towerIndexMotor.getEncoder();
    private SparkLimitSwitch noteSensor = bottomIndexMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    private boolean spinBottom = true;

    public IndexerIOReal() {
        towerIndexMotor.restoreFactoryDefaults();
        bottomIndexMotor.restoreFactoryDefaults();

        towerIndexMotor.setInverted(false);
        bottomIndexMotor.setInverted(false);
        
        noteSensor.enableLimitSwitch(false);

        towerIndexMotor.setSmartCurrentLimit(CurrentLimits.tower);
        bottomIndexMotor.setSmartCurrentLimit(CurrentLimits.handoff);

        towerEncoder.setVelocityConversionFactor(IndexerConstants.kVelocityConversionFactor);
        bottomEncoder.setVelocityConversionFactor(IndexerConstants.kVelocityConversionFactor);

        bottomEncoder.setPosition(0);
        towerEncoder.setPosition(0);

        towerIndexMotor.setIdleMode(IdleMode.kBrake);
        bottomIndexMotor.setIdleMode(IdleMode.kCoast);

        towerIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        towerIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        towerIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);
        towerIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        towerIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
        towerIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
        towerIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

        bottomIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        bottomIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        bottomIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);
        bottomIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        bottomIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
        bottomIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
        bottomIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

        towerIndexMotor.burnFlash();
        bottomIndexMotor.burnFlash();
    }

    @Override
    public void setIndexerSpeed(double speed) {
        if (spinBottom) {
            bottomIndexMotor.set(speed);
        }
        towerIndexMotor.set(speed);
    }

    @Override
    public void canSpinBottom(boolean spin) {
        spinBottom = spin;
    }

    @Override
    public void setHandoffSpeed(double speed){
        if(spinBottom){
            bottomIndexMotor.set(speed);
        }
    }

    @Override
    public void setTowerSpeed(double speed){
        towerIndexMotor.set(speed);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {    
        inputs.bottomMotorSpeed = bottomEncoder.getVelocity();
        inputs.bottomMotorCurrent = bottomIndexMotor.getOutputCurrent();
        inputs.bottomMotorVoltage = bottomIndexMotor.getAppliedOutput();

        inputs.towerMotorSpeed = towerEncoder.getVelocity();
        inputs.towerMotorCurrent = towerIndexMotor.getOutputCurrent();
        inputs.towerMotorVoltage = towerIndexMotor.getAppliedOutput();

        inputs.sensorActivated = noteSensor.isPressed();
    }
}
