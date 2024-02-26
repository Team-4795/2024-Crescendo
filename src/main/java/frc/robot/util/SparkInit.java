package frc.robot.util;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.MAXSwerve.DriveConstants.ModuleConstants;

public class SparkInit {
    public static void swerveDriving(CANSparkBase motor, RelativeEncoder encoder, SparkPIDController controller) {

        applyParameter(
                () -> controller.setFeedbackDevice(encoder),
                () -> true,
                "Couldn't set PID feedback device");

        applyParameter(
                () -> encoder.setAverageDepth(2),
                () -> encoder.getAverageDepth() == 2,
                "Encoder average depth mismatch");

        applyParameter(
                () -> encoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor),
                () -> encoder.getPositionConversionFactor() == ModuleConstants.kDrivingEncoderPositionFactor,
                "Driving Position Conversion Factor Failed");

        applyParameter(
                () -> encoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor),
                () -> encoder.getVelocityConversionFactor() == ModuleConstants.kDrivingEncoderVelocityFactor,
                "Driving Velocity Conversion Factor Failed");

        // Set the PID gains for the driving motor. Note these are example gains, and
        // you
        // may need to tune them for your own robot!

        applyParameter(
                () -> controller.setP(ModuleConstants.kDrivingP),
                () -> controller.getP() == ModuleConstants.kDrivingP,
                "Incorrect Spark PIDF value - kP");

        applyParameter(
                () -> controller.setI(ModuleConstants.kDrivingI),
                () -> controller.getI() == ModuleConstants.kDrivingI,
                "Incorrect Spark PIDF value - kI");

        applyParameter(
                () -> controller.setD(ModuleConstants.kDrivingD),
                () -> controller.getD() == ModuleConstants.kDrivingD,
                "Incorrect Spark PIDF value - kD");

        applyParameter(
                () -> controller.setFF(ModuleConstants.kDrivingFF),
                () -> controller.getFF() == ModuleConstants.kDrivingFF,
                "Incorrect Spark PIDF value - kFF");

        applyParameter(
                () -> controller.setOutputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput),
                () -> controller.getOutputMax() == ModuleConstants.kDrivingMaxOutput
                        && controller.getOutputMin() == ModuleConstants.kDrivingMinOutput,
                "Incorret Output Max/Min");

        applyParameter(
                () -> motor.setIdleMode(ModuleConstants.kDrivingMotorIdleMode),
                () -> motor.getIdleMode() == ModuleConstants.kDrivingMotorIdleMode,
                "Wrong Driving Motor Idle Mode");

        applyParameter(
                () -> motor.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit),
                () -> true,
                "Failure to Set Driving Motor Current Limit");

        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);

        for (int i = 0; i < Constants.paramApplyAttemptLimit; i++) {
            REVLibError status;
            Timer.delay(0.2);
            status = motor.burnFlash();
            Timer.delay(0.2);
            if (status == REVLibError.kOk)
                break;
        }
    }

    public static void swerveTurning(CANSparkBase motor, AbsoluteEncoder encoder, SparkPIDController controller) {
        applyParameter(
                () -> controller.setFeedbackDevice(encoder),
                () -> true,
                "Couldn't set PID feedback device");

        applyParameter(
                () -> encoder.setAverageDepth(2),
                () -> encoder.getAverageDepth() == 2,
                "Encoder average depth mismatch");

        // in radians and radians per second
        applyParameter(
                () -> encoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor),
                () -> encoder.getPositionConversionFactor() == ModuleConstants.kTurningEncoderPositionFactor,
                "Turning Position Conversion Factor Failed");

        applyParameter(
                () -> encoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor),
                () -> encoder.getVelocityConversionFactor() == ModuleConstants.kTurningEncoderVelocityFactor,
                "Turning Velocity Conversion Factor Failed");

        applyParameter(
                () -> encoder.setInverted(ModuleConstants.kTurningEncoderInverted),
                () -> encoder.getInverted() == ModuleConstants.kTurningEncoderInverted,
                "Failed to Set Correct Turning Motor Inversion");

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        applyParameter(
                () -> controller.setPositionPIDWrappingEnabled(true),
                () -> controller.getPositionPIDWrappingEnabled() == true,
                "Turning PID No Wrapping :(");

        applyParameter(
                () -> controller.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput),
                () -> true,
                "Failure to set max position PID output");

        applyParameter(
                () -> controller.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput),
                () -> true,
                "Failure to set min position PID output");

        // Set the PID gains for the turning motor. Note these are example gains, and
        // you
        // may need to tune them for your own robot!
        applyParameter(
                () -> controller.setP(ModuleConstants.kTurningP),
                () -> controller.getP() == ModuleConstants.kTurningP,
                "Incorrect Spark PIDF value - kP");

        applyParameter(
                () -> controller.setI(ModuleConstants.kTurningI),
                () -> controller.getI() == ModuleConstants.kTurningI,
                "Incorrect Spark PIDF value - kI");

        applyParameter(
                () -> controller.setD(ModuleConstants.kTurningD),
                () -> controller.getD() == ModuleConstants.kTurningD,
                "Incorrect Spark PIDF value - kD");

        applyParameter(
                () -> controller.setFF(ModuleConstants.kTurningFF),
                () -> controller.getFF() == ModuleConstants.kTurningFF,
                "Incorrect Spark PIDF value - kFF");

        applyParameter(
                () -> controller.setOutputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput),
                () -> controller.getOutputMax() == ModuleConstants.kTurningMaxOutput
                        && controller.getOutputMin() == ModuleConstants.kTurningMinOutput,
                "Incorret Output Max/Min");

        applyParameter(
                () -> motor.setIdleMode(ModuleConstants.kTurningMotorIdleMode),
                () -> motor.getIdleMode() == ModuleConstants.kTurningMotorIdleMode,
                "Wrong Turning Motor Idle Mode");

        applyParameter(
                () -> motor.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit),
                () -> true,
                "Failure to Set Turning Motor Current Limit");

        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 1000);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        for (int i = 0; i < Constants.paramApplyAttemptLimit; i++) {
            REVLibError status;
            Timer.delay(0.2);
            status = motor.burnFlash();
            Timer.delay(0.2);
            if (status == REVLibError.kOk)
                break;
        }
    }

    public static REVLibError applyParameter(Supplier<REVLibError> parameterSetter,
            BooleanSupplier parameterCheckSupplier, String errorMessage) {
        if (Constants.currentMode == Mode.SIM)
            return parameterSetter.get();
        REVLibError status = REVLibError.kError;

        for (int i = 0; i < Constants.paramApplyAttemptLimit; i++) {
            status = parameterSetter.get();
            if (parameterCheckSupplier.getAsBoolean() && status == REVLibError.kOk)
                break;
            Timer.delay(Constants.paramApplyTimemout);
        }

        if (status != REVLibError.kOk)
            System.out.printf("%s - %s", errorMessage, status.toString());
        return status;
    }

}
