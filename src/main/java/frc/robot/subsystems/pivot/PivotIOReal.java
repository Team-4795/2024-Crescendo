package frc.robot.subsystems.pivot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

public class PivotIOReal implements PivotIO {
  private CANSparkFlex pivotLeft = new CANSparkFlex(PivotConstants.leftCanID, MotorType.kBrushless);
  private CANSparkFlex pivotRight = new CANSparkFlex(PivotConstants.rightCanID, MotorType.kBrushless);
  private AbsoluteEncoder encoder = pivotLeft.getAbsoluteEncoder(Type.kDutyCycle);

  public PivotIOReal() {
    pivotRight.restoreFactoryDefaults();
    pivotLeft.restoreFactoryDefaults();

    pivotLeft.setSmartCurrentLimit(30);
    pivotRight.setSmartCurrentLimit(30);

    pivotLeft.setIdleMode(IdleMode.kBrake);
    pivotRight.setIdleMode(IdleMode.kBrake);

    pivotRight.follow(pivotLeft, true);

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
    inputs.pivotAppliedVolts = pivotRight.getBusVoltage();
    inputs.pivotPositionRads = encoder.getPosition();
    inputs.pivotVelocityRadPerSec = encoder.getVelocity();
  }

}
