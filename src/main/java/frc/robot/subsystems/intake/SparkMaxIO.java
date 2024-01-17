package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;



public class SparkMaxIO implements IntakeIO{
    private final CANSparkMax frontRoller = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax backRoller = new CANSparkMax(10, MotorType.kBrushless);
    private final RelativeEncoder frontEncoder = frontRoller.getEncoder();
    private final RelativeEncoder backEncoder = backRoller.getEncoder();

    public SparkMaxIO() {
    frontRoller.setInverted(false);
    backRoller.setInverted(true);

    backRoller.follow(frontRoller);

    frontRoller.burnFlash();
    backRoller.burnFlash();

    frontRoller.setSmartCurrentLimit(40);
    backRoller.setSmartCurrentLimit(40);
    }
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocity = frontEncoder.getVelocity();
        inputs.voltage = frontRoller.getOutputCurrent();
    }

    @Override
    public void setMotorSpeed(double speed) {
        frontRoller.set(speed);
    }
    
    } 

