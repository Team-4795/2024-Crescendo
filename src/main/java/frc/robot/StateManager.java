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
    private boolean enabled = true;

    public enum State {
        Stow(StateConstants.stow),
        GroundIntake(StateConstants.groundIntake),
        SourceIntake(StateConstants.sourceIntake),
        ScoreAmp(StateConstants.scoreAmp),
        ScoreSpeaker(StateConstants.scoreSpeaker),
        Back(StateConstants.back),
        RampUp(StateConstants.rampUp),
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
        Shooter.getInstance().setShootingSpeedRPM(
                this.state.setpoint.topShooterMotor(), this.state.setpoint.bottomShooterMotor());
        Intake.getInstance().setIntakeSpeed(this.state.setpoint.intake());
        Indexer.getInstance().setIndexerSpeed(this.state.setpoint.indexer());
        if (state == State.Init) {
            Pivot.getInstance().reset();
        } else {
            Pivot.getInstance().setGoal(this.state.setpoint.pivot());
        }
    }

    public static StateManager getInstance() {
        if (mInstance == null) {
            mInstance = new StateManager();
        }

        return mInstance;
    }

    public void setMutable(boolean on) {
        this.enabled = on;
    }

    public void periodic() {

        Logger.recordOutput("StateManager/State", state.toString());
    }
}
