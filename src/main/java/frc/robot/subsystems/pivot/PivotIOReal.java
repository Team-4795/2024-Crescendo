package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.CurrentLimits;

public class PivotIOReal implements PivotIO {
  private CANSparkFlex pivotLeft = new CANSparkFlex(PivotConstants.leftCanID, MotorType.kBrushless);
  private CANSparkFlex pivotRight = new CANSparkFlex(PivotConstants.rightCanID, MotorType.kBrushless);
  private RelativeEncoder motorEncoderLeft = pivotLeft.getEncoder();
  private RelativeEncoder motorEncoderRight = pivotLeft.getEncoder();

  private AbsoluteEncoder encoder = pivotRight.getAbsoluteEncoder();
  // private DutyCycleEncoder encoder = new DutyCycleEncoder(9);
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

    encoder.setPositionConversionFactor(PivotConstants.positionConversionFactor);
    encoder.setVelocityConversionFactor(PivotConstants.velocityConversionFactor);

    pivotLeft.setIdleMode(IdleMode.kBrake);
    pivotRight.setIdleMode(IdleMode.kBrake);

    pivotRight.follow(pivotLeft, true);

    // encoder.setAverageDepth(2);

    motorEncoderLeft.setAverageDepth(2);
    motorEncoderLeft.setMeasurementPeriod(20);

    motorEncoderRight.setAverageDepth(2);
    motorEncoderRight.setMeasurementPeriod(20);

    motorEncoderLeft.setPositionConversionFactor(Math.PI * 2 / PivotConstants.gearing);
    motorEncoderLeft.setVelocityConversionFactor(Math.PI * 2 / 60 / PivotConstants.gearing);

    motorEncoderRight.setPositionConversionFactor(Math.PI * 2 / PivotConstants.gearing);
    motorEncoderRight.setVelocityConversionFactor(Math.PI * 2 / 60 / PivotConstants.gearing);
    for(int i = 0; i < 10; i++){
      pivotLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
      pivotLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
      pivotLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
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
      pivotRight.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
    }
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
    // volts = MathUtil.clamp(volts, -12, 12);
    inputVolts = volts;
    pivotLeft.setVoltage(volts);
  }

  // private double getAbsolutePosition() {
  //   double rotation = -encoder.getAbsolutePosition() * PivotConstants.positionConversionFactor - 3.1293;
  //   if(rotation < -1){
  //     rotation += 2 * Math.PI;
  //   }
  //   return rotation;
  // }

  private double getAbsolutePosition() {
    double rotation = encoder.getPosition();
    if(rotation < -2){
      rotation += 2 * Math.PI;
    } else if (rotation > 4) {
      rotation -= 2 * Math.PI;
    }
    return rotation;
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.pivotInputVolts = inputVolts;
    inputs.pivotAppliedVolts = pivotLeft.getAppliedOutput() * pivotLeft.getBusVoltage();
    inputs.pivotPositionRads = getAbsolutePosition();
    inputs.pivotVelocityRads = encoder.getVelocity();
    inputs.pivotCurrent = pivotLeft.getOutputCurrent();
    inputs.pivotMotorPositionRads = avg(motorEncoderLeft.getPosition(), motorEncoderRight.getPosition());
    inputs.pivotMotorVelocityRadPerSec = avg(motorEncoderLeft.getVelocity(), motorEncoderRight.getVelocity());

    Logger.recordOutput("Pivot/Motor Enc Left", motorEncoderLeft.getPosition());
    Logger.recordOutput("Pivot/Motor Enc Right", motorEncoderRight.getPosition());

  }

  private double avg(double x1, double x2) {
    return (x1 + x2) / 2;
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

  @Override
  public void resetEncoders() {
    motorEncoderLeft.setPosition(getAbsolutePosition());
    motorEncoderRight.setPosition(getAbsolutePosition());
  }
}
