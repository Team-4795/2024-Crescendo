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

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants.FieldConstants;

import java.util.Set;
import java.util.List;
import java.util.Objects;
import java.util.stream.Stream;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class NoteVisualizer {
  // private static final Translation3d blueSpeaker = new Translation3d(0.225, 5.55, 2.1);
  // private static final Translation3d redSpeaker = new Translation3d(16.317, 5.55, 2.1);
  private static final double shotSpeed = 12.0; // Meters per sec
  private static Supplier<Pose3d> pivotPoseSupplier = () -> new Pose3d();

  private static final List<Translation2d> autoNotes = new ArrayList<>();

  public static void setPivotPoseSupplier(Supplier<Pose3d> supplier) {
    pivotPoseSupplier = supplier;
  }

  public static void showAutoNotes() {
    if (autoNotes.isEmpty()) {
      Logger.recordOutput("NoteVisualizer/StagedNotes");
    }
    // Show auto notes
    Stream<Translation2d> presentNotes = autoNotes.stream().filter(Objects::nonNull);
    Logger.recordOutput(
        "NoteVisualizer/StagedNotes",
        presentNotes
            .map(
                translation ->
                    new Pose3d(
                        translation.getX(),
                        translation.getY(),
                        Units.inchesToMeters(1.0),
                        new Rotation3d()))
            .toArray(Pose3d[]::new));
  }

  /** Add all notes to be shown at the beginning of auto */
  public static void resetAutoNotes() {
    for (int i = FieldConstants.StagingLocations.spikeTranslations.length - 1; i >= 0; i--) {
      autoNotes.add(flip(FieldConstants.StagingLocations.spikeTranslations[i]));
    }
    for (int i = FieldConstants.StagingLocations.centerlineTranslations.length - 1; i >= 4; i--) {
      autoNotes.add(
          flip(FieldConstants.StagingLocations.centerlineTranslations[i]));
    }
  }

  public static double flip(double x) {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      return FieldConstants.fieldLength - x;
    } else {
      return x;
    }
  }

  public static Translation2d flip(Translation2d x) {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      return new Translation2d(flip(x.getX()), x.getY());
    } else {
      return x;
    }
  }

  public static Command shoot() {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  final Pose3d startPose = pivotPoseSupplier.get();

                  // final double duration =
                  //     startPose.getTranslation().getDistance(endPose.getTranslation()) / shotSpeed;
                  final double duration = 1;
                  final Pose3d endPose = startPose.transformBy(new Transform3d(shotSpeed * duration, 0, 0, new Rotation3d()));
                  final Timer timer = new Timer();
                  timer.start();

                  return Commands.run(
                          () -> {
                            Logger.recordOutput(
                                "NoteVisualizer",
                                new Pose3d[] {
                                  startPose.interpolate(endPose, timer.get() / duration)
                                });
                          })
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(
                          () -> {
                            Logger.recordOutput("NoteVisualizer", new Pose3d[] {});
                          });
                },
                Set.of())
            .ignoringDisable(true));
  }
}