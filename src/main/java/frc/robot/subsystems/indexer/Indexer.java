package frc.robot.subsystems.indexer;

import java.beans.Statement;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateManager;
import frc.robot.StateManager.State;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.Constants.IndexerSetpoints;

public class Indexer extends SubsystemBase {
    
    private IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged(); 
    private double indexerSpeed = 0.0;
    private boolean shouldSpin = false;
    private boolean overrideStoring = false;
    private boolean isAuto = false;

    public static boolean currentStoring = false;
    private CircularBuffer<Double> currents = new CircularBuffer<>(IndexerConstants.bufferSize);

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

    public void setSpin(boolean on){
        shouldSpin = on;
    }

    public void setAutoMode(boolean on){
        isAuto = on;
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
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        double averageCurrent = this.averageCurrent();
        currents.addLast(Double.valueOf(inputs.leftMotorCurrent));

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

        Logger.recordOutput("Indexer/Average current", averageCurrent);
        Logger.recordOutput("Indexer/Storing (based on current)", currentStoring);
    }

    private double averageCurrent(){
        double sum = 0;
        for(int i = 0; i < IndexerConstants.bufferSize; i++){
            sum += currents.get(i);
        }
        return (sum / IndexerConstants.bufferSize);
    }
}