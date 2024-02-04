package frc.robot;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.Setpoint;
import frc.robot.Constants.StateConstants;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;

public class StateManager {
    private static StateManager mInstance;
    
    public State state = State.Stow;

    public enum State {
        Stow(StateConstants.stow),
        GroundIntake(StateConstants.groundIntake),
        SourceIntake(StateConstants.sourceIntake),
        ScoreAmp(StateConstants.scoreAmp),
        ScoreSpeaker(StateConstants.scoreSpeaker);

        Setpoint setpoint;

        State(Setpoint setpoint) {
            this.setpoint = setpoint;
        }
    }

    public void setState(State state) {
        this.state = state;
        this.setSetpoints();
    }

    public void setSetpoints() {
        Shooter.getInstance().setShootingSpeed(this.state.setpoint.shooter());
        Pivot.getInstance().setGoal(this.state.setpoint.pivot());
        Intake.getInstance().setIntakeSpeed(this.state.setpoint.intake());
        Indexer.getInstance().setIndexerSpeed(this.state.setpoint.indexer());
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
