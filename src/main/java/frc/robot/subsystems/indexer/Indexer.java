package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    
    private IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged(); 
    private double indexerSpeed = 0.0;

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

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        io.setIndexerSpeed(indexerSpeed);
    }

}