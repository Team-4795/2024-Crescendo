package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.autoPaths.AS_GP123;
import frc.robot.autoPaths.AS_GP1234;
import frc.robot.autoPaths.AutoPath;
import frc.robot.autoPaths.Close3PAuto;
import frc.robot.autoPaths.testingPath;
import frc.robot.commands.AutoCommands;

public class AutoSelector {
    private final LoggedDashboardChooser<AutoPath> chooser = new LoggedDashboardChooser<>("Auto Selector");

    AutoCommands autoCommands;

    public AutoSelector() {
        autoCommands = new AutoCommands();

        chooser.addOption("Close3PAuto", new Close3PAuto());
        chooser.addDefaultOption("Testing auto", new testingPath());
        chooser.addOption("AS GP1234", new AS_GP1234());
        chooser.addOption("As GP123", new AS_GP123());
        
    }

    public Command getSelected() {
        return chooser.get().load(autoCommands);
    }
}