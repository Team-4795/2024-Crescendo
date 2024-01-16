package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterIOReal implements ShooterIO {
    private CANSparkMax angleMotor = new CANSparkMax(12, MotorType.kBrushless);
    private CANSparkMax rightShooterMotor = new CANSparkMax(13,MotorType.kBrushless);
    private CANSparkMax leftShooterMotor = new CANSparkMax(14, MotorType.kBrushless);
    private RelativeEncoder encoder =  angleMotor.getEncoder();
    private RelativeEncoder leftShooterEncoder = leftShooterMotor.getEncoder();
    private RelativeEncoder rightShooterEncoder = rightShooterMotor.getEncoder();

    public ShooterIOReal(){
        angleMotor.setSmartCurrentLimit(30);
        rightShooterMotor.setSmartCurrentLimit(30);
        leftShooterMotor.setSmartCurrentLimit(30);
        angleMotor.burnFlash();
        rightShooterMotor.burnFlash();
        leftShooterMotor.burnFlash();
        leftShooterMotor.setInverted(true);
        encoder.setPosition(0);
    }

    @Override
    public void rotateAngleMotor(double speed) {
        angleMotor.set(speed);
    }

    @Override
    public void runShooterMotors(double speed) {
        leftShooterMotor.set(speed);
        rightShooterMotor.set(speed);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.angleMotorAppliedVolts = angleMotor.getBusVoltage();
        inputs.angleMotorRelativePosition = encoder.getPosition();
        inputs.angleMotorVelocityRadPerSec = encoder.getVelocity();
        inputs.shooterMotorAppliedVolts = leftShooterMotor.getBusVoltage();
        inputs.shooterMotorVelocityRadPerSec = leftShooterEncoder.getVelocity();
    }

    
}