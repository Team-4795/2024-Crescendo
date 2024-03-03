package frc.robot.subsystems.pivot;

import java.security.MessageDigest;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class PivotVisualizer {
    Mechanism2d measMechanism;
    MechanismRoot2d measMechanismRoot;
    MechanismLigament2d measPivot;

    Mechanism2d setpointMechanism;
    MechanismRoot2d setpointMechanismRoot;
    MechanismLigament2d setpointPivot;
    
    Pose3d pivotPose;

    public PivotVisualizer() {
        measMechanism = new Mechanism2d(2, 1.5);
        measMechanismRoot = measMechanism.getRoot("Measured Pivot", 0.5, 0.5);
        measPivot = measMechanismRoot.append(new MechanismLigament2d("Measured Pivot", 0.68, 0, 6, new Color8Bit(Color.kDarkRed)));

        setpointMechanism = new Mechanism2d(2, 1.5);
        setpointMechanismRoot = setpointMechanism.getRoot("Setpoint Pivot", 0.5, 0.5);
        setpointPivot = setpointMechanismRoot.append(new MechanismLigament2d("Setpoint Pivot", 0.68, 0, 6, new Color8Bit(Color.kDarkOrange)));
    }

    public void update(double pivotAngle, double setpointAngle) {
        measPivot.setAngle(pivotAngle);
        setpointPivot.setAngle(setpointAngle);
        
        Logger.recordOutput("PivotMechanism2d/Measured", measMechanism);
        Logger.recordOutput("PivotMechanism2d/Setpoint", setpointMechanism);

        pivotPose = new Pose3d(0.0, 0.254, 0.215, new Rotation3d((0.62)-Units.degreesToRadians(pivotAngle), 0, 0)).rotateBy(new Rotation3d(0, 0, -Math.PI / 2));
        Logger.recordOutput("PivotMechanism3d", pivotPose);
    }
}