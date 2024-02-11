package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;

public class ShooterIOReal implements ShooterIO {
    
    //private CANSparkMax rightShooterMotor = new CANSparkMax(ShooterConstants.rightCanID,MotorType.kBrushless);
    private TalonFX rightShooterMotor = new TalonFX(ShooterConstants.rightCanID);
    private TalonFX leftShooterMotor = new TalonFX(ShooterConstants.leftCanID);

    private TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    public ShooterIOReal(){

        talonFXConfig.Slot0.kP = 0.05;
        talonFXConfig.Slot0.kI = 0;
        talonFXConfig.Slot0.kD = 0;
        talonFXConfig.Slot0.kS = 0;
        talonFXConfig.Slot0.kV = 0;

        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = 30;
        talonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.SupplyCurrentLimit = 30;    

        talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // talonFXConfig.MotionMagic.MotionMagicAcceleration = 100;
        // talonFXConfig.MotionMagic.MotionMagicCruiseVelocity = 10;

        talonFXConfig.Audio.BeepOnBoot = true;

        rightShooterMotor.setControl(new Follower(leftShooterMotor.getDeviceID(), true));

        leftShooterMotor.clearStickyFaults();
        rightShooterMotor.clearStickyFaults();

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
        // set velocity to certain rps, add 0.5 V to overcome gravity
        //leftShooterMotor.setControl(m_request.withVelocity(speed).withFeedForward(0.2));

        leftShooterMotor.set(MathUtil.clamp(speed, -1, 1));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterMotorAppliedVolts = leftShooterMotor.getMotorVoltage().getValueAsDouble();
        inputs.shooterMotorVelocityRPM = leftShooterMotor.getVelocity().getValueAsDouble();
    }

    
}