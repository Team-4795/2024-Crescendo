package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class PivotIOReal implements PivotIO {
  private CANSparkFlex pivotLeft = new CANSparkFlex(PivotConstants.leftCanID, MotorType.kBrushless);
  private CANSparkFlex pivotRight = new CANSparkFlex(PivotConstants.rightCanID, MotorType.kBrushless);
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(9);

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
    Logger.recordOutput("Pivot/DistancePerRotation", encoder.getDistancePerRotation());

    inputs.pivotAppliedVolts = pivotRight.getBusVoltage();
    inputs.pivotPositionRads = encoder.getAbsolutePosition();
    // inputs.pivotPositionRads = encoder.getPosition();
    // inputs.pivotVelocityRadPerSec = encoder.getVelocity();
  }
}
