package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;


public interface IndexerIO {
    
   @AutoLog
   public static class IndexerIOInputs {
    public double motorSpeed;
    public double motorPos;
    public double motorCurrent;
    public double motorVoltage;
   }

   public default void updateInputs(IndexerIOInputs inputs) {}
   public default void setIndexerSpeed(double speed) {}
   public default double getLeftMotorVoltage() { return 0; }
   public default double getRightMotorVoltage() { return 0; }
   public default double getLeftMotorCurrent() { return 0; }
   public default double getRightMotorCurrent() { return 0; }
}
