package frc.robot;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;

public class StateManager {
    private static StateManager mInstance;
    
    public State state;

    public enum State {
        Stow(
            Constants.StateConstants.pivotAngleStow,
            Constants.StateConstants.intakeSpeedStow,
            Constants.StateConstants.indexerSpeedStow,
            Constants.StateConstants.shooterSpeedStow
        );

        double pivotAngle;
        double intakeSpeed;
        double indexerSpeed;
        double shooterSpeed;

        State(double pivotAngle, double intakeSpeed, double indexerSpeed, double shooterSpeed) {
            this.pivotAngle = pivotAngle;
            this.intakeSpeed = intakeSpeed;
            this.indexerSpeed = indexerSpeed;
            this.shooterSpeed = shooterSpeed;
        }
    }

    public void setState(State state) {
        this.state = state;
    }

    public void setSetpoints() {
        Shooter.getInstance().setShootingSpeed(this.state.shooterSpeed);
        Pivot.getInstance().setGoal(this.state.pivotAngle);
        Intake.getInstance().setIntakeSpeed(this.state.intakeSpeed);
        Indexer.getInstance().setIndexerSpeed(this.state.indexerSpeed);
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
