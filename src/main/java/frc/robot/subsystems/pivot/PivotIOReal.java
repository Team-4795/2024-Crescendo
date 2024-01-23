package frc.robot.subsystems.pivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;


import com.revrobotics.CANSparkLowLevel.MotorType;

public class PivotIOReal implements PivotIO {
  private CANSparkMax pivotLeft = new CANSparkMax(PivotConstants.leftCanID, MotorType.kBrushless);
  private CANSparkMax pivotRight = new CANSparkMax(PivotConstants.rightCanID, MotorType.kBrushless);
  private RelativeEncoder encoder = pivotLeft.getEncoder();
 
 
 public PivotIOReal() {
    pivotLeft.setSmartCurrentLimit(30);
    pivotRight.setSmartCurrentLimit(30);
    pivotRight.setInverted(true);

    pivotRight.follow(pivotLeft);

    pivotRight.burnFlash();
    pivotLeft.burnFlash();
    encoder.setPosition(0);
  }

  @Override
  public void rotatePivot(double speed) {
    pivotLeft.set(speed);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.pivotAppliedVolts = pivotLeft.getBusVoltage();
    inputs.pivotRelativePosition = encoder.getPosition();
    inputs.pivotVelocityRadPerSec = encoder.getVelocity();
  }

}
