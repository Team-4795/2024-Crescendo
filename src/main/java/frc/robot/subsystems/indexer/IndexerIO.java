package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;


public interface IndexerIO {
    
   @AutoLog
   public static class IndexerIOInputs {
     public double bottomMotorSpeed = 0.0;
     public double bottomMotorCurrent = 0.0;
     public double bottomMotorVoltage = 0.0;
 
     public double towerMotorSpeed = 0.0;
     public double towerMotorCurrent = 0.0;
     public double towerMotorVoltage = 0.0;
     public boolean sensorActivated = false;
   }

   public default void canSpinBottom(boolean spin) {}
   public default void updateInputs(IndexerIOInputs inputs) {}
   public default void setIndexerSpeed(double speed) {}
   public default void setHandoffSpeed(double speed) {}
   public default void setTowerSpeed(double speed) {}
}
