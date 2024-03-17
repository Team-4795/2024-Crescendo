package frc.robot.subsystems.pivot;

// import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.CurrentLimits;

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

    pivotLeft.enableVoltageCompensation(12);
    pivotRight.enableVoltageCompensation(12);

    pivotLeft.setSmartCurrentLimit(CurrentLimits.pivot);
    pivotRight.setSmartCurrentLimit(CurrentLimits.pivot);

    // encoder.setPositionConversionFactor(PivotConstants.positionConversionFactor);
    // encoder.setVelocityConversionFactor(PivotConstants.velocityConversionFactor);

    pivotLeft.setIdleMode(IdleMode.kBrake);
    pivotRight.setIdleMode(IdleMode.kBrake);

    pivotRight.follow(pivotLeft, true);

    motorEncoder.setAverageDepth(2);
    motorEncoder.setMeasurementPeriod(20);

    motorEncoder.setPositionConversionFactor(Math.PI * 2 / PivotConstants.gearing);
    motorEncoder.setVelocityConversionFactor(Math.PI * 2 / 60 / PivotConstants.gearing);
    // motorEncoder.setPosition(encoder.get() * -Math.PI * 2 + 0.18);

    pivotLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    pivotLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    pivotLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);
    pivotLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    pivotLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    pivotLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
    pivotLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

    // pivotRight.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    pivotRight.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    pivotRight.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);
    pivotRight.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    pivotRight.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
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
    double rotation = -encoder.getAbsolutePosition() * PivotConstants.positionConversionFactor + 4.187 - 3.151- 0.048;
    if(rotation < -1){
      rotation += 2 * Math.PI;
    }
    return rotation;
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.pivotInputVolts = inputVolts;
    inputs.pivotAppliedVolts = pivotLeft.getAppliedOutput() * pivotLeft.getBusVoltage();
    inputs.pivotPositionRads = getAbsolutePosition();
    inputs.pivotCurrent = pivotLeft.getOutputCurrent();

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
