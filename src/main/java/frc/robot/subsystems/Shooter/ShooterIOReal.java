package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterIOReal implements ShooterIO {
    
    private CANSparkMax rightShooterMotor = new CANSparkMax(ShooterConstants.rightCanID,MotorType.kBrushless);
    private CANSparkMax leftShooterMotor = new CANSparkMax(ShooterConstants.leftCanID, MotorType.kBrushless);
    private RelativeEncoder leftShooterEncoder = leftShooterMotor.getEncoder();
    private SparkPIDController controller;

    public ShooterIOReal(){
        controller = leftShooterMotor.getPIDController();
        controller.setFeedbackDevice(leftShooterEncoder);
        controller.setP(ShooterConstants.shooterP);
        controller.setI(0);
        controller.setD(0);
        rightShooterMotor.setSmartCurrentLimit(30);
        leftShooterMotor.setSmartCurrentLimit(30);
        leftShooterMotor.setInverted(true);
        rightShooterMotor.follow(leftShooterMotor);   
        rightShooterMotor.burnFlash();
        leftShooterMotor.burnFlash();
    }


    @Override
    public void runShooterMotors(double speed) {
        leftShooterMotor.set(speed);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterMotorAppliedVolts = leftShooterMotor.getBusVoltage();
        inputs.shooterMotorVelocityRPM = leftShooterEncoder.getVelocity();
    }

    
}