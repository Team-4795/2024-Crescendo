package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterIOReal implements ShooterIO {

    private CANSparkMax rightShooterMotor = new CANSparkMax(ShooterConstants.rightCanID,MotorType.kBrushless);
    private CANSparkMax leftShooterMotor = new CANSparkMax(ShooterConstants.leftCanID, MotorType.kBrushless);
    private RelativeEncoder leftShooterEncoder = leftShooterMotor.getEncoder();
    private RelativeEncoder rightShooterEncoder = rightShooterMotor.getEncoder();
    private SparkPIDController controller;

    public ShooterIOReal(){
        controller = leftShooterMotor.getPIDController();
        controller.setFeedbackDevice(leftShooterEncoder);
        controller.setP(ShooterConstants.shooterP);
        controller.setI(0);
        controller.setD(0);
        rightShooterMotor.follow(leftShooterMotor);
        rightShooterMotor.setSmartCurrentLimit(30);
        leftShooterMotor.setSmartCurrentLimit(30);
        leftShooterMotor.setInverted(true);     
        rightShooterMotor.burnFlash();
        leftShooterMotor.burnFlash();
    }


    @Override
    public void runShooterMotors(double speed) {
        controller.setReference(speed, ControlType.kVelocity);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterMotorAppliedVolts = leftShooterMotor.getBusVoltage();
        inputs.shooterMotorVelocityRPM = leftShooterEncoder.getVelocity();
    }

    
}