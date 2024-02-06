package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.autoPaths.AutoPath;
import frc.robot.commands.AutoCommands;

public class AutoSelector {
    private final LoggedDashboardChooser<AutoPath> chooser = new LoggedDashboardChooser<>("Auto Selector");

    AutoCommands autoCommands;

    public AutoSelector() {
        autoCommands = new AutoCommands();

        //chooser.addOption("Free 3 Hybrid MHM", new Free3HybridMHM());
    }

    public Command getSelected() {
        return chooser.get().load(autoCommands);
    }
}