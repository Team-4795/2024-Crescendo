package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;


public interface IndexerIO {
    
   @AutoLog
   public static class IndexerIOInputs {
    public double motorSpeed = 0.0;
    public double motorCurrent = 0.0;
    public double motorVoltage = 0.0;
   }

   public default void updateInputs(IndexerIOInputs inputs) {}
   public default void setIndexerSpeed(double speed) {}

   public default double getLeftMotorVelocity() { return 0; }
   public default double getRightMotorVelocity() { return 0; }
   public default double getLeftMotorCurrent() { return 0; }
   public default double getRightMotorCurrent() { return 0; }
}
