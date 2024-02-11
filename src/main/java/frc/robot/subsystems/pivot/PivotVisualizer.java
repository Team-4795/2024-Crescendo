package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class PivotVisualizer {
    Mechanism2d mechanism;
    MechanismRoot2d mechanismRoot;
    MechanismLigament2d pivot;
    MechanismLigament2d wrist;
    Pose3d pivotPose;

    public PivotVisualizer(Color color) {

        mechanism = new Mechanism2d(2, 1.5);
        mechanismRoot = mechanism.getRoot("Pivot", 0.5, 0.5);
        pivot = mechanismRoot.append(new MechanismLigament2d("Pivot", 0.68, 0, 6, new Color8Bit(color)));

    }

    public void update(double pivotAngle) {
        pivot.setAngle(pivotAngle);
        Logger.recordOutput("PivotMechanism2d", mechanism);

        pivotPose = new Pose3d(0.25, -0.2, 0.2, new Rotation3d(0, -Math.toRadians(pivotAngle), 0));
        Logger.recordOutput("PivotMechanism3d", pivotPose);
    }
}