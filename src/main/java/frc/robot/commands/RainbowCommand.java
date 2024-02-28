// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.leds.LEDs;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class RainbowCommand extends Command {
    LEDs leds = LEDs.getInstance();
    double hue;
    DoubleSupplier speed;

    public RainbowCommand(DoubleSupplier speed) {
        this.speed = speed;

        this.hue = 0;

        addRequirements(leds);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Hue is 0-180
        this.hue += speed.getAsDouble() / 20.0;

        for (int i = 0; i < leds.getLength(); i++) {
            double offset = (double) i / (double) leds.getLength();
            double color = (180.0 * (hue + offset));

            color = (color - 1 + 180) % 180;

            if (color < 0.0) color += 180.0;

            leds.setHSVIndex(i, (int) color, 255, 255);
        }

        leds.setOutput();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}