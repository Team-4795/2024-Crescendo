package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

public class IndexerIOReal implements IndexerIO {
    private CANSparkMax leftIndexMotor = new CANSparkMax(IndexerConstants.leftCanID, MotorType.kBrushless);
    private CANSparkMax rightIndexMotor = new CANSparkMax(IndexerConstants.rightCanID, MotorType.kBrushless);
    private RelativeEncoder leftEncoder = leftIndexMotor.getEncoder();
    private RelativeEncoder rightEncoder = rightIndexMotor.getEncoder();
    private SparkLimitSwitch noteSensor = leftIndexMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    private boolean spinBottom = true;

    public IndexerIOReal() {
        rightIndexMotor.restoreFactoryDefaults();
        leftIndexMotor.restoreFactoryDefaults();
        
        noteSensor.enableLimitSwitch(false);

        rightIndexMotor.setSmartCurrentLimit(25);
        leftIndexMotor.setSmartCurrentLimit(25);

        rightEncoder.setVelocityConversionFactor(IndexerConstants.kVelocityConversionFactor);
        leftEncoder.setVelocityConversionFactor(IndexerConstants.kVelocityConversionFactor);

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        rightIndexMotor.setIdleMode(IdleMode.kBrake);
        leftIndexMotor.setIdleMode(IdleMode.kCoast);

        rightIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        rightIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
        rightIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
        rightIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        rightIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        rightIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        rightIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);

        leftIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        leftIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
        leftIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
        leftIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        leftIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        leftIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        leftIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);

        rightIndexMotor.burnFlash();
        leftIndexMotor.burnFlash();
    }

    @Override
    public void setIndexerSpeed(double speed) {
        if (spinBottom) {
            leftIndexMotor.set(speed);
        }
        rightIndexMotor.set(speed);
    }

    @Override
    public void canSpinBottom(boolean spin) {
        spinBottom = spin;
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {    
        inputs.leftMotorSpeed = leftEncoder.getVelocity();
        inputs.leftMotorCurrent = leftIndexMotor.getOutputCurrent();
        inputs.leftMotorVoltage = leftIndexMotor.getAppliedOutput();

        inputs.rightMotorSpeed = rightEncoder.getVelocity();
        inputs.rightMotorCurrent = rightIndexMotor.getOutputCurrent();
        inputs.rightMotorVoltage = rightIndexMotor.getAppliedOutput();

        inputs.sensorActivated = noteSensor.isPressed();
    }
}
