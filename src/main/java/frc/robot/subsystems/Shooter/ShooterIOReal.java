package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterIOReal implements ShooterIO {
    
    //private CANSparkMax rightShooterMotor = new CANSparkMax(ShooterConstants.rightCanID,MotorType.kBrushless);
    private TalonFX rightShooterMotor = new TalonFX(ShooterConstants.rightCanID);
    private TalonFX leftShooterMotor = new TalonFX(ShooterConstants.leftCanID);

    private TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    // private RelativeEncoder leftShooterEncoder = leftShooterMotor.getEncoder();
    // private RelativeEncoder rightShooterEncoder = rightShooterMotor.getEncoder();
    private SparkPIDController controller;

    public ShooterIOReal(){
        talonFXConfig.CurrentLimits.SupplyCurrentLimit = 30;
        // controller = leftShooterMotor.getPIDController();
        // controller.setFeedbackDevice(leftShooterEncoder);
        // controller.setP(ShooterConstants.shooterP);
        talonFXConfig.Slot0.kP = 0;
        talonFXConfig.Slot0.kI = 0;
        talonFXConfig.Slot0.kD = 0;

        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = 30;

        talonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.SupplyCurrentLimit = 30;

        // controller.setI(0);
        // controller.setD(0);
        leftShooterMotor.setInverted(true);     

        talonFXConfig.MotionMagic.MotionMagicAcceleration = 100;
        talonFXConfig.MotionMagic.MotionMagicCruiseVelocity = 10;

        StatusCode response = leftShooterMotor.getConfigurator().apply(talonFXConfig);
        if (!response.isOK()) {
            System.out.println(
                    "Talon ID "
                            + leftShooterMotor.getDeviceID()
                            + " failed config with error "
                            + response.toString());
        }
        response = rightShooterMotor.getConfigurator().apply(talonFXConfig);
        if (!response.isOK()) {
            System.out.println(
                    "Talon ID "
                            + rightShooterMotor.getDeviceID()
                            + " failed config with error "
                            + response.toString());
        }
    }


    @Override
    public void runShooterMotors(double speed) {
        leftShooterMotor.set(speed);
        rightShooterMotor.set(speed);
        
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterMotorAppliedVolts = leftShooterMotor.getSupplyVoltage().getValueAsDouble();
        inputs.shooterMotorVelocityRPM = leftShooterMotor.getVelocity().getValueAsDouble();
    }

    
}