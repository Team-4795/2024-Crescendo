package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerSetpoints;
import frc.robot.subsystems.pivot.Pivot;

public class Indexer extends SubsystemBase {
    
    private IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged(); 
    private double indexerSpeed = 0.0;
    private boolean overrideStoring = false;
    private boolean currentSpeedStoring = false;

    public static boolean currentStoring = false;
    private CircularBuffer<Double> currents = new CircularBuffer<>(IndexerConstants.bufferSize);
    private double lastMeasuredSpeed = 0.0;

    private static Indexer instance;

    public static Indexer getInstance(){
        return instance;
    }

    public static Indexer initialize(IndexerIO IndexIo){
        if(instance == null){
            instance = new Indexer(IndexIo);
        }
        return instance;
    }

    private Indexer(IndexerIO IndexIo) {
        io = IndexIo;
        io.updateInputs(inputs);

        for(int i = 0; i < IndexerConstants.bufferSize; i++){
            currents.addLast(Double.valueOf(0));
        }
    }

    public void setIndexerSpeed(double motorValue) {
        indexerSpeed = motorValue;
    }

    public Command reverse() {
        return startEnd(
            () -> setIndexerSpeed(IndexerSetpoints.reverse),
            () -> setIndexerSpeed(0)
        );
    }

    public Command forwards() {
        return startEnd(
            () -> setIndexerSpeed(IndexerSetpoints.shoot),
            () -> setIndexerSpeed(0)
        );
    }

    public Command slowReverse() {
        return startEnd(
            () -> setIndexerSpeed(IndexerSetpoints.slowReverse),
            () -> setIndexerSpeed(0)
        );
    }

    public Command overrideStoring() {
        return startEnd(
            () -> overrideStoring = true,
            () -> overrideStoring = false
        );
    }

    public boolean isStoring() {
        // Flip the value if overrideStoring is true
        return inputs.sensorActivated ^ overrideStoring;
    }

    public boolean handoff(){
        return currentStoring;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        double averageCurrent = this.averageCurrent();
        currents.addLast(Double.valueOf(inputs.bottomMotorCurrent));
        Logger.recordOutput("Indexer/Last Measured Speed", lastMeasuredSpeed);
        double absoluteAcceleration = Math.abs(inputs.bottomMotorSpeed) - Math.abs(lastMeasuredSpeed);
        lastMeasuredSpeed = inputs.bottomMotorSpeed;

        if (Pivot.getInstance().getPosition() < 1.0) {
            io.canSpinBottom(true);
        } else {
            io.canSpinBottom(false);
        }

        io.setIndexerSpeed(indexerSpeed);

        if(averageCurrent > IndexerConstants.currentThreshold){
            currentStoring = true;
        } else {
            currentStoring = false;
        }

        if(averageCurrent > IndexerConstants.currentThreshold && absoluteAcceleration < 0) { 
            currentSpeedStoring = true;
        }
        else {
            currentSpeedStoring = false;
        }
        
        Logger.recordOutput("Indexer/Average current", averageCurrent);
        Logger.recordOutput("Indexer/Acceleration", inputs.bottomMotorSpeed - lastMeasuredSpeed);
        Logger.recordOutput("Indexer/Absolute Acceleration", absoluteAcceleration);
        Logger.recordOutput("Indexer/Storing (based on current)", currentStoring);
        Logger.recordOutput("Indexer/Storing2 (based on current and speed)", currentStoring);
    }

    private double averageCurrent(){
        double sum = 0;
        for(int i = 0; i < IndexerConstants.bufferSize; i++){
            sum += currents.get(i);
        }
        return (sum / IndexerConstants.bufferSize);
    }
}