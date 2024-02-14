package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class PivotIOReal implements PivotIO {
  private CANSparkFlex pivotLeft = new CANSparkFlex(PivotConstants.leftCanID, MotorType.kBrushless);
  private CANSparkFlex pivotRight = new CANSparkFlex(PivotConstants.rightCanID, MotorType.kBrushless);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(9);
  private RelativeEncoder motorEncoder = pivotLeft.getEncoder();

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

    encoder.setDistancePerRotation(-Math.PI * 2);
    encoder.setPositionOffset(-0.66);

    motorEncoder.setPositionConversionFactor(Math.PI * 2 / PivotConstants.gearing);
    motorEncoder.setVelocityConversionFactor(Math.PI * 2 / 60 / PivotConstants.gearing);
    motorEncoder.setPosition(encoder.getAbsolutePosition());

    pivotLeft.burnFlash();
    pivotRight.burnFlash();

  }

  @Override
  public void rotatePivot(double speed) {
    pivotLeft.set(speed);
  }

  @Override
  public void setVoltage(double volts) {
    pivotLeft.setVoltage(volts);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.pivotAppliedVolts = pivotLeft.getAppliedOutput();
    inputs.pivotPositionRads = encoder.getDistance();

    inputs.pivotMotorPositionRads = motorEncoder.getPosition();
    inputs.pivotMotorVelocityRadPerSec = motorEncoder.getVelocity();
  }
}
