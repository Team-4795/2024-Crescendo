package frc.robot;

import org.littletonrobotics.junction.Logger;

public class StateManager {
    private static StateManager mInstance;
    
    public State state;

    public enum State {
        Speaker(
            Constants.StateConstants.pivotAngleSpeaker, 
            Constants.StateConstants.intakeSpeedSpeaker, 
            Constants.StateConstants.indexerSpeedSpeaker
        );

        double pivotAngle;
        double intakeSpeed;
        double indexerSpeed;

        State(double pivotAngle, double intakeSpeed, double indexerSpeed) {
            this.pivotAngle = pivotAngle;
            this.intakeSpeed = intakeSpeed;
            this.indexerSpeed = indexerSpeed;
        }
    }

    public void setState(State state) {
        this.state = state;
    }

    public static StateManager getInstance() {
        if (mInstance == null) { 
            mInstance = new StateManager();
        }
        
        return mInstance;
    }

    public void periodic() {
        Logger.recordOutput("StateManager/State", state.toString());
    }
}
