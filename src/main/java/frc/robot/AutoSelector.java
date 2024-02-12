package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.autoPaths.AutoPath;
import frc.robot.autoPaths.Close3PAuto;
import frc.robot.autoPaths.testingPath;
import frc.robot.commands.AutoCommands;

public class AutoSelector {
    private final LoggedDashboardChooser<AutoPath> chooser = new LoggedDashboardChooser<>("Auto Selector");

    AutoCommands autoCommands;

    public AutoSelector() {
        autoCommands = new AutoCommands();

        chooser.addDefaultOption("Close3PAuto", new Close3PAuto());
        chooser.addOption("Testing auto", new testingPath());
    }

    public Command getSelected() {
        return chooser.get().load(autoCommands);
    }
}