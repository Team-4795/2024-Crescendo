package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    
    CANSparkMax indexMotor = new CANSparkMax(69, MotorType.kBrushless);

    public Indexer() {
        indexMotor.setSmartCurrentLimit(30);
        indexMotor.burnFlash();
    }

    public void spin(double motorValue) {
        indexMotor.set(motorValue);
    }

}