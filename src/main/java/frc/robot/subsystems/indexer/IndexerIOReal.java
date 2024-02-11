package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IndexerIOReal implements IndexerIO {
    private CANSparkMax leftIndexMotor = new CANSparkMax(IndexerConstants.canId, MotorType.kBrushless);
    private CANSparkMax rightIndexMotor = new CANSparkMax(IndexerConstants.canId, MotorType.kBrushless);
    private RelativeEncoder leftEncoder = leftIndexMotor.getEncoder();
    private RelativeEncoder rightEncoder = rightIndexMotor.getEncoder();

    public IndexerIOReal() {
        rightIndexMotor.setSmartCurrentLimit(30);
        leftIndexMotor.setSmartCurrentLimit(30);

        rightEncoder.setPositionConversionFactor(IndexerConstants.kPositionConversionFactor);
        leftEncoder.setVelocityConversionFactor(IndexerConstants.kVelocityConversionFactor);

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        rightIndexMotor.follow(leftIndexMotor, false);

        rightIndexMotor.burnFlash();
        leftIndexMotor.burnFlash();

    }

    @Override
    public void setIndexerSpeed(double speed) {
        leftIndexMotor.set(speed);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.motorSpeed = leftEncoder.getVelocity();
        inputs.motorPos = leftEncoder.getPosition();
        inputs.motorCurrent = leftIndexMotor.getOutputCurrent();
        inputs.motorVoltage = leftIndexMotor.getBusVoltage();
    }

    @Override
    public double getLeftMotorCurrent() {
        return leftIndexMotor.getOutputCurrent();
    }
    @Override
    public double getRightMotorCurrent() {
        return rightIndexMotor.getOutputCurrent();
    }
    @Override
    public double getLeftMotorVoltage() {
        return leftIndexMotor.getBusVoltage();
    }
    @Override
    public double getRightMotorVoltage() {
        return rightIndexMotor.getBusVoltage();
    }
}
