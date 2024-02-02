package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ShooterVisualizer {
    String name;
    Mechanism2d mechanism2d;
    MechanismRoot2d mechanismIsRoot;
    MechanismLigament2d shooter;

    public ShooterVisualizer(String name, Color color) {
        this.name = name;

        mechanism2d = new Mechanism2d(0, 0);
        mechanismIsRoot = mechanism2d.getRoot("Shooter", 0, 0);
        shooter = mechanismIsRoot.append(new MechanismLigament2d("Shooter", 0, 0, 0, new Color8Bit(color)));

    }

    public void update(double speed) {

    }
}
