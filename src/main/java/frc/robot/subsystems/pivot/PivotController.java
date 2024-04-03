package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants;
import frc.robot.util.LoggedProfiledPIDController;

public final class PivotController {
    private LoggedProfiledPIDController normalController = new LoggedProfiledPIDController(
        PivotConstants.kP,
        PivotConstants.kI, 
        PivotConstants.kD,
        PivotConstants.constraints);

    private final TrapezoidProfile profile = new TrapezoidProfile(PivotConstants.constraints);
    private TrapezoidProfile.State lastState = new TrapezoidProfile.State();

    private final LinearSystem<N2, N1, N1> armPlant =
        LinearSystemId.createSingleJointedArmSystem(DCMotor.getNeoVortex(2), PivotConstants.inertia, PivotConstants.gearing);

    private final KalmanFilter<N2, N1, N1> observer =
        new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            armPlant,
            VecBuilder.fill(0.2, 0.2), // How accurate we
            // think our model is, in radians and radians/sec
            VecBuilder.fill(0.01), // How accurate we think our encoder position
            // data is. In this case we very highly trust our encoder position reading.
            0.02);

    private final LinearQuadraticRegulator<N2, N1, N1> lqr =
        new LinearQuadraticRegulator<>(
            armPlant,
            VecBuilder.fill(0.01, 0.15), // Position, Velocity weight (Lower is more penalized)
            VecBuilder.fill(12.0), // Voltage weight
            0.02);

    private final LinearSystemLoop<N2, N1, N1> loop =
        new LinearSystemLoop<>(armPlant, lqr, observer, 12.0, 0.02);

    public PivotController() {
        if (Constants.useLQR) {
            Logger.recordOutput("Pivot/LQR P", loop.getController().getK().get(0, 0));
            Logger.recordOutput("Pivot/LQR D", loop.getController().getK().get(0, 1));
        }

        normalController.setIntegratorRange(-1, 1);

        // normalController.enableContinuousInput(0, 2 * Math.PI);
    }

    public double calculate(double measurement, double goal) {
        if (Constants.useLQR) {
            Logger.recordOutput("Pivot/LQR Pos", loop.getXHat(0));
            Logger.recordOutput("Pivot/LQR Vel", loop.getXHat(1));

            // SmartDashboard.putNumber("Arm estimate pos", observer.getXhat(0) / PI2);
            // SmartDashboard.putNumber("Arm estimate vel", observer.getXhat(1) / PI2);

            lastState = profile.calculate(0.02, lastState, new TrapezoidProfile.State(goal, 0));

            loop.setNextR(lastState.position, lastState.velocity);
            loop.correct(VecBuilder.fill(measurement));
            loop.predict(0.02);

            return loop.getU(0);
        } else {
            TrapezoidProfile.State goalState = new TrapezoidProfile.State(goal, 0);
            return normalController.calculate(measurement, goalState, PivotConstants.constraints);
        }
    }

    public void reset(double measurement) {
        if (Constants.useLQR) {
            lastState = new TrapezoidProfile.State(measurement, 0);
            loop.reset(VecBuilder.fill(measurement, 0));
        } else {
            normalController.reset(measurement);
        }
    }

    public State getSetpoint() {
        if (Constants.useLQR) {
            return lastState;
        } else {
            return normalController.getSetpoint();
        }
    }

    public void setPID(double kp, double ki, double kd) {
        normalController.setPID(kp, ki, kd);
    }
}