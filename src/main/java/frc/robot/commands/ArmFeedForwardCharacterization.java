// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.PolynomialRegression;
import frc.robot.util.MultipleLinearRegression;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class ArmFeedForwardCharacterization extends Command {
  private static final double START_DELAY_SECS = 2.0;
  private static final double RAMP_VOLTS_PER_SEC = 0.2;

  private FeedForwardCharacterizationData data;
  private final Consumer<Double> voltageConsumer;
  private final Supplier<Double> velocitySupplier;
  private final Supplier<Double> positionSupplier;

  private final Timer timer = new Timer();

  /** Creates a new FeedForwardCharacterization command. */
  public ArmFeedForwardCharacterization(
      Subsystem subsystem, Consumer<Double> voltageConsumer, Supplier<Double> velocitySupplier, Supplier<Double> positionSupplier) {
    addRequirements(subsystem);
    this.voltageConsumer = voltageConsumer;
    this.velocitySupplier = velocitySupplier;
    this.positionSupplier = positionSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    data = new FeedForwardCharacterizationData();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < START_DELAY_SECS) {
      voltageConsumer.accept(0.0);
    } else {
      double voltage = (timer.get() - START_DELAY_SECS) * RAMP_VOLTS_PER_SEC;
      voltageConsumer.accept(voltage);
      data.add(velocitySupplier.get(), voltage, positionSupplier.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    voltageConsumer.accept(0.0);
    timer.stop();
    data.print();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static class FeedForwardCharacterizationData {
    private final List<Double> velocityData = new LinkedList<>();
    private final List<Double> voltageData = new LinkedList<>();
    private final List<Double> positionData = new LinkedList<>();

    public void add(double velocity, double voltage, double position) {
      if (Math.abs(velocity) > 1E-4) {
        velocityData.add(Math.abs(velocity));
        voltageData.add(Math.abs(voltage));
        positionData.add(position);
      }
    }

    public void print() {
      if (velocityData.size() == 0 || voltageData.size() == 0 || positionData.size() == 0) {
        return;
      }

      System.out.println(voltageData);
      System.out.println(positionData);
      System.out.println(velocityData);

      PolynomialRegression regression =
          new PolynomialRegression(
              velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
              voltageData.stream().mapToDouble(Double::doubleValue).toArray(),
              1);

      System.out.println("Simple FF Characterization Results:");
      System.out.println("\tCount=" + Integer.toString(velocityData.size()) + "");
      System.out.println(String.format("\tR2=%.5f", regression.R2()));
      System.out.println(String.format("\tkS=%.5f", regression.beta(0)));
      System.out.println(String.format("\tkV=%.5f", regression.beta(1)));

      double[] y = velocityData.subList(1, velocityData.size())
          .stream().mapToDouble(Double::doubleValue).toArray();

      double[][] x = new double[velocityData.size() - 1][3];

      for (int i = 0; i < velocityData.size() - 1; i++) {
        x[i] = new double[] { velocityData.get(i), voltageData.get(i), Math.signum(velocityData.get(i)) };
      }

      MultipleLinearRegression regression2 = new MultipleLinearRegression(x, y);

      double alpha = regression2.beta(0);
      double beta = regression2.beta(1);
      double gamma = regression2.beta(2);

      double ks = -gamma / beta;
      double kv = (1 - alpha) / beta;
      double ka = (alpha - 1) * 0.02 / (beta * Math.log(alpha));

      if (alpha <= 0.0 || Math.abs(beta) < 1e-5) {
        System.out.println("Warning: Data is outside of expected bounds, results may be invalid.");
      }

      System.out.println("Full FF Characterization Results:");
      System.out.println("\tCount=" + Integer.toString(velocityData.size()) + "");
      System.out.println(String.format("\tR2=%.5f", regression2.R2()));
      System.out.println(String.format("\tkS=%.5f", ks));
      System.out.println(String.format("\tkV=%.5f", kv));
      System.out.println(String.format("\tkA=%.5f", ka));
    }
  }
}
