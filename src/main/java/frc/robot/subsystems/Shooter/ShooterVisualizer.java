package frc.robot.subsystems.Shooter;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ShooterVisualizer {
    Mechanism2d mechanism2d;
    MechanismRoot2d mechanismIsRoot;
    MechanismLigament2d shooter;

    public ShooterVisualizer(String name, Color color) {
        mechanism2d = new Mechanism2d(0, 0);
        mechanismIsRoot = mechanism2d.getRoot("Shooter", 0, 0);
        shooter = mechanismIsRoot.append(new MechanismLigament2d("Shooter", 0, 0, 0, new Color8Bit(color)));

    }

    public void update() {
        Logger.recordOutput("Shooter2d", mechanism2d);

        var shooterPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
        Logger.recordOutput("Shooter3D", shooterPose);
    }
}
