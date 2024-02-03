package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IndexerIOReal implements IndexerIO {

    private CANSparkMax indexMotor = new CANSparkMax(IndexerConstants.canId, MotorType.kBrushless);
    private RelativeEncoder encoder = indexMotor.getEncoder();

    public IndexerIOReal() {
        indexMotor.setSmartCurrentLimit(30);

        encoder.setPositionConversionFactor(IndexerConstants.kPositionConversionFactor);
        encoder.setVelocityConversionFactor(IndexerConstants.kVelocityConversionFactor);

        indexMotor.burnFlash();
        encoder.setPosition(0);
    }

    @Override
    public void setIndexerSpeed(double speed) {
        indexMotor.set(speed);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.motorSpeed = encoder.getVelocity();
        inputs.motorPos = encoder.getPosition();
    }

    
    
}
