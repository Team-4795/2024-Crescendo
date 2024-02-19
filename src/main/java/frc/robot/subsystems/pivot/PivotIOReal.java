package frc.robot.subsystems.pivot;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class PivotIOReal implements PivotIO {
  private CANSparkFlex pivotLeft = new CANSparkFlex(PivotConstants.leftCanID, MotorType.kBrushless);
  private CANSparkFlex pivotRight = new CANSparkFlex(PivotConstants.rightCanID, MotorType.kBrushless);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(9);
  private RelativeEncoder motorEncoder = pivotLeft.getEncoder();
  private double inputVolts = 0.0;

  public PivotIOReal() {
    pivotRight.restoreFactoryDefaults();
    pivotLeft.restoreFactoryDefaults();

    pivotLeft.setSmartCurrentLimit(30);
    pivotRight.setSmartCurrentLimit(30);

    // encoder.setPositionConversionFactor(PivotConstants.positionConversionFactor);
    // encoder.setVelocityConversionFactor(PivotConstants.velocityConversionFactor);

    pivotLeft.setIdleMode(IdleMode.kBrake);
    pivotRight.setIdleMode(IdleMode.kBrake);

    pivotRight.follow(pivotLeft, true);

    // encoder.reset();

    motorEncoder.setPositionConversionFactor(Math.PI * 2 / PivotConstants.gearing);
    motorEncoder.setVelocityConversionFactor(Math.PI * 2 / 60 / PivotConstants.gearing);
    motorEncoder.setPosition(encoder.get() * -Math.PI * 2 + 0.18);

    pivotLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    pivotLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    pivotLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    pivotLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);

    pivotRight.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    pivotRight.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    pivotRight.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    pivotRight.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);

    pivotLeft.burnFlash();
    pivotRight.burnFlash();
  }

  @Override
  public void rotatePivot(double speed) {
    inputVolts = speed * 12;
    pivotLeft.set(speed);
  }

  @Override
  public void setVoltage(double volts) {
    inputVolts = volts;
    pivotLeft.setVoltage(volts);
  }

  private double getAbsolutePosition() {
    return encoder.get() * -Math.PI * 2 + 4.175;
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.pivotInputVolts = inputVolts;
    inputs.pivotAppliedVolts = pivotLeft.getAppliedOutput() * pivotLeft.getBusVoltage();
    inputs.pivotPositionRads = getAbsolutePosition();

    inputs.pivotMotorPositionRads = motorEncoder.getPosition();
    inputs.pivotMotorVelocityRadPerSec = motorEncoder.getVelocity();
  }
  
  @Override
  public void setIdleMode(boolean idleMode) {
    if (idleMode) {
      pivotLeft.setIdleMode(IdleMode.kBrake);
      pivotRight.setIdleMode(IdleMode.kBrake);
    } else {
      pivotLeft.setIdleMode(IdleMode.kCoast);
      pivotRight.setIdleMode(IdleMode.kCoast);
    }
  }
}
