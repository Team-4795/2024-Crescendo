package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateManager;
import frc.robot.StateManager.State;

public class Indexer extends SubsystemBase {
    
    private IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged(); 
    private double indexerSpeed = 0.0;
    private boolean shouldSpin = false;

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
    }

    public void setIndexerSpeed(double motorValue) {
        indexerSpeed = motorValue;
    }

    public void setSpin(boolean on){
        shouldSpin = on;
    }

    public void reverse() {
        indexerSpeed *= -1;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        if(StateManager.getInstance().state == State.GroundIntake || shouldSpin){
            io.setIndexerSpeed(indexerSpeed);
        } else {
            io.setIndexerSpeed(0);
        }

    }

}