package frc.robot;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.Setpoint;
import frc.robot.Constants.StateConstants;
import frc.robot.commands.ScoreSpeaker;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;

public class StateManager {
    private static StateManager mInstance;

    public State state = State.Stow;
    // private State overrideState = null;
    // private boolean override = false;
    private boolean enabled = true;

    public enum State {
        Stow(StateConstants.stow),
        GroundIntake(StateConstants.groundIntake),
        SourceIntake(StateConstants.sourceIntake),
        ScoreAmp(StateConstants.scoreAmp),
        ScoreSpeaker(StateConstants.scoreSpeaker),
        Init(StateConstants.init);

        Setpoint setpoint;

        State(Setpoint setpoint) {
            this.setpoint = setpoint;
        }
    }

    public void setState(State state) {
        if (enabled) {
            this.state = state;
            this.setSetpoints();
        }
    }

    public void setSetpoints() {
        Setpoint desiredSetpoint = this.state.setpoint;

        if(desiredSetpoint.topShooterMotor() != null && desiredSetpoint.bottomShooterMotor() != null){
            Shooter.getInstance().setShootingSpeedRPM(
                desiredSetpoint.topShooterMotor(), desiredSetpoint.bottomShooterMotor());
        }
        
        if(desiredSetpoint.intake() != null) {
            Intake.getInstance().setIntakeSpeed(desiredSetpoint.intake());
        }
        
        if(desiredSetpoint.indexer() != null){
            Indexer.getInstance().setIndexerSpeed(desiredSetpoint.indexer());
        }
        
        if (state == State.Init) {
            Pivot.getInstance().reset();
        } else if (desiredSetpoint.pivot() != null) {
            Pivot.getInstance().setGoal(desiredSetpoint.pivot());
        }
    }

    public static StateManager getInstance() {
        if (mInstance == null) {
            mInstance = new StateManager();
        }

        return mInstance;
    }

    public void setMutable(boolean mutable) {
        this.enabled = mutable;
    }

    public void periodic() {
        Logger.recordOutput("StateManager/State", state.toString());
    }
}
