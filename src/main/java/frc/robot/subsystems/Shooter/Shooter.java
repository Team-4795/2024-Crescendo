package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private CANSparkMax angleMotor = new CANSparkMax(12, MotorType.kBrushless);
    private CANSparkMax ShooterMotor = new CANSparkMax(13,MotorType.kBrushless);

    public Shooter(){
        
    }

    
}
