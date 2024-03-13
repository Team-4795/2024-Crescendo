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

package frc.robot;





import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * The Constants class provides a csonvenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Mode of the robot, set to Mode.REPLAY for replay
  public static final Mode currentMode = Mode.fromState();
  public static final boolean tuningMode = true;
  public static final boolean hasVision = true;
  public static final int tryConfigCount = 5;
  public static final double configDelay = 0.1;
  public static final int paramApplyAttemptLimit = 5;
  public static final double paramApplyTimemout = 0.05;

  public static final class CurrentLimits {
    public static final int pivot = 60;
    public static final int drive = 60;
    public static final int turning = 20;
    public static final int intakeVortex = 60;
    public static final int intakeKraken = 80;
    public static final int tower = 25;
    public static final int handoff = 25;
    public static final int shooter = 80;
  }

  public static final class PivotSetpoints {
    public static final double speaker = 0.56;
    public static final double amp = 1.15;
    public static final double stow = 0.08;
    public static final double intake = 0.3;
    public static final double source = 0.96;
  }

  public static final class IndexerSetpoints {
    public static final double shoot = 1.0;
    // public static final double amp = 1.0;
    public static final double reverse = -1.0;
    public static final double slowReverse = -0.2;
  }

  public static final class ShooterSetpoints {
    public static final double speakerTop = -4500;
    public static final double speakerBottom = 4500;

    public static final double ampTop = 500;
    public static final double ampBottom = 500;

    public static final double reverseTop = 1000;
    public static final double reverseBottom = -1000;

    public static final double slowReverseTop = 650;
    public static final double slowReverseBottom = -650;
  }

  public static final class IntakeSetpoints {
    public static final double intake = -0.6;
    public static final double reverse = 0.6;
    public static final double slowReverse = 0.4;
  }

  //shooter, indexer, intake measured in motor output, pivot measured in radians
  public record Setpoint(Double topShooterMotor, Double bottomShooterMotor, Double pivot, Double indexer, Double intake) {}

  public class StateConstants {
    public static final Setpoint stow = new Setpoint(0.0,0.0, 0.08, 0.0, 0.0);
    public static final Setpoint groundIntake = new Setpoint(0.0,0.0, 0.16, 1.0, -0.7);
    public static final Setpoint sourceIntake = new Setpoint(750.0, -750.0, 0.96, -0.5, 0.0);
    public static final Setpoint scoreAmp = new Setpoint(600.0,600.0, 1.0, 1.0, 0.0);
    public static final Setpoint scoreSpeaker = new Setpoint(-5000.0,5000.0,0.6, 1.0, 0.0);  
    public static final Setpoint load = new Setpoint(null, null, null, -1.0, 0.0);
    public static final Setpoint reverse = new Setpoint(1000.0, -1000.0, null, -1.0, 0.7);
    public static final Setpoint counter = new Setpoint(null, null, null, 1.0, 0.6);
    public static final Setpoint init = new Setpoint(0.0, 0.0, 0.0, 0.0, 0.0);
  }


  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY;

    static Mode fromState() {
      if (Robot.isReal()) {
        return REAL;
      } else {
        return SIM;
      }
    }
  }

  public static final class OIConstants{
    public static final double kAxisDeadband = 0.1;
    public static final CommandXboxController driverController = new CommandXboxController(0);
    public static final CommandXboxController operatorController = new CommandXboxController(1);
  }

  public static final class FieldConstants {
    public static final double speakerHeight = 1.98; //meters
    public static final double fieldLength = Units.inchesToMeters(651.223);

    public static final class StagingLocations {
      public static final double centerlineX = fieldLength / 2.0;

      // need to update
      public static final double centerlineFirstY = Units.inchesToMeters(29.638);
      public static final double centerlineSeparationY = Units.inchesToMeters(66);
      public static final double spikeX = Units.inchesToMeters(114);
      // need
      public static final double spikeFirstY = Units.inchesToMeters(161.638);
      public static final double spikeSeparationY = Units.inchesToMeters(57);

      public static final Translation2d[] centerlineTranslations = new Translation2d[5];
      public static final Translation2d[] spikeTranslations = new Translation2d[3];

      static {
        for (int i = 0; i < centerlineTranslations.length; i++) {
          centerlineTranslations[i] =
              new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
        }
      }

      static {
        for (int i = 0; i < spikeTranslations.length; i++) {
          spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
        }
      }
    }
  }

  public static class PathFindingConstants {
    public static final Pose2d blueAmp = new Pose2d(7.5,4.5, new Rotation2d());
    public static final Pose2d redAmp = new Pose2d(7.5, 4.5, new Rotation2d());
  }

  public static class AutoConstants {
    public static final double closePivotSetpoint = 3; //tune later
    
    /* CAN IDs (for 2 motor subsystems lower id is on left facing robot forward)
    * Swerve 2-9
    * Intake 10
    * Pivot 11-12
    * Indexer 13-14 
    * Shooter 15-16
    */
  }
}
