package frc.robot.subsystems.pivot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;

public class PivotIOReal implements PivotIO {
  private CANSparkFlex pivotLeft = new CANSparkFlex(PivotConstants.leftCanID, MotorType.kBrushless);
  private CANSparkFlex pivotRight = new CANSparkFlex(PivotConstants.rightCanID, MotorType.kBrushless);
  private RelativeEncoder motorEncoder = pivotLeft.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);
  // private AbsoluteEncoder encoder = pivotRight.getAbsoluteEncoder();
  private DutyCycleEncoder encoder = new DutyCycleEncoder(9);
  private double inputVolts = 0.0;

  public PivotIOReal() {
    pivotRight.restoreFactoryDefaults();
    pivotLeft.restoreFactoryDefaults();

    pivotLeft.setCANTimeout(250);
    pivotRight.setCANTimeout(250);

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
    // motorEncoder.setPosition(encoder.get() * -Math.PI * 2 + 0.18);

    pivotLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    pivotLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
    pivotLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);
    pivotLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    pivotLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    pivotLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    pivotLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);

    // pivotRight.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    pivotRight.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
    pivotRight.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);
    pivotRight.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    pivotRight.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    pivotRight.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

    pivotLeft.burnFlash();
    pivotRight.burnFlash();

    pivotLeft.setCANTimeout(0);
    pivotRight.setCANTimeout(0);
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
    return encoder.getAbsolutePosition() * PivotConstants.positionConversionFactor;
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.pivotInputVolts = inputVolts;
    inputs.pivotAppliedVolts = pivotLeft.getAppliedOutput() * pivotLeft.getBusVoltage();

    if (Math.abs(inputs.pivotPositionRads - getAbsolutePosition()) < 0.1) {
      inputs.pivotPositionRads = getAbsolutePosition();
    }

    // Cut off weird jumps
    if (Math.abs(motorEncoder.getPosition()) < 2.0) {
      inputs.pivotMotorPositionRads = motorEncoder.getPosition();
    }
    
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
