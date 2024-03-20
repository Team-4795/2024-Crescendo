package frc.robot.subsystems.vision.intakeCam;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.IntakeCamVisionIOInputsAutoLogged;;

public class IntakeCamVision extends SubsystemBase {
    private static IntakeCamVisionIO io;
    private IntakeCamVisionIOInputsAutoLogged inputs = new IntakeCamVisionIOInputsAutoLogged();

    private static IntakeCamVision instance;

    public static IntakeCamVision getInstance() {
        return instance;
    }

    public static IntakeCamVision initialize() {
        if (instance == null) {
            instance = new IntakeCamVision(io);
        }
        return instance;
    }

    private IntakeCamVision(IntakeCamVisionIO intakeCamVisionIO) {
        io = intakeCamVisionIO;
    }

    public double getIntakeCamYaw() {
        return inputs.camYaw;
    }

    public boolean intakeCamHasTargets() {
        return inputs.hasTargets;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake Cam", inputs);
    }
    
}
