package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;


public interface IndexerIO {
    
   @AutoLog
   public static class IndexerIOInputs {
    public double motorSpeed = 0.0;
    public double motorPos = 0.0;
   } 

   public default void updateInputs(IndexerIOInputs inputs) {}
   public default void setIndexerSpeed(double speed) {}

}
