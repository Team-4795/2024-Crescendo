package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;


public interface IndexerIO {
    
   @AutoLog
   public static class IndexerIOInputs {
     public double leftMotorSpeed = 0.0;
     public double leftMotorCurrent = 0.0;
     public double leftMotorVoltage = 0.0;
 
     public double rightMotorSpeed = 0.0;
     public double rightMotorCurrent = 0.0;
     public double rightMotorVoltage = 0.0;
     public boolean sensorActivated = false;
   }

   public default void canSpinBottom(boolean spin) {}
   public default void updateInputs(IndexerIOInputs inputs) {}
   public default void setIndexerSpeed(double speed) {}
}
