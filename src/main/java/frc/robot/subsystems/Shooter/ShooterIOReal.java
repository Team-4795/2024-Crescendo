package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterIOReal implements ShooterIO {
    
    private CANSparkMax rightShooterMotor = new CANSparkMax(13,MotorType.kBrushless);
    private CANSparkMax leftShooterMotor = new CANSparkMax(14, MotorType.kBrushless);
    private RelativeEncoder leftShooterEncoder = leftShooterMotor.getEncoder();
    private RelativeEncoder rightShooterEncoder = rightShooterMotor.getEncoder();

    public ShooterIOReal(){
        
        rightShooterMotor.setSmartCurrentLimit(30);
        leftShooterMotor.setSmartCurrentLimit(30);
        
        rightShooterMotor.burnFlash();
        leftShooterMotor.burnFlash();
        leftShooterMotor.setInverted(true);
        
    }

  

    @Override
    public void runShooterMotors(double speed) {
        leftShooterMotor.set(speed);
        rightShooterMotor.set(speed);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterMotorAppliedVolts = leftShooterMotor.getBusVoltage();
        inputs.shooterMotorVelocityRadPerSec = leftShooterEncoder.getVelocity();
    }

    
}