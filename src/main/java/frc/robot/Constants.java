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





import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
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
  public static final boolean hasVision = false;
  public static final int tryConfigCount = 5;
  public static final double configDelay = 0.1;

  public static final class PivotSetpoints {
    public static final double speaker = 0.6;
    public static final double amp = 1.1;
    public static final double stow = 0.08;
    public static final double intake = 0.15;
    public static final double source = 0.96;
  }

  public static final class IndexerSetpoints {
    public static final double shoot = 1.0;
    // public static final double amp = 1.0;
    public static final double reverse = -1.0;
    public static final double slowReverse = -0.2;
  }

  public static final class ShooterSetpoints {
    public static final double speakerTop = -5000;
    public static final double speakerBottom = 5000;

    public static final double ampTop = 600;
    public static final double ampBottom = 600;

    public static final double reverseTop = 1000;
    public static final double reverseBottom = -1000;

    public static final double slowReverseTop = 150;
    public static final double slowReverseBottom = -150;
  }

  public static final class IntakeSetpoints {
    public static final double intake = -0.8;
    public static final double reverse = 0.8;
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
    public static final CommandPS4Controller driverController = new CommandPS4Controller(0);
    public static final CommandXboxController operatorController = new CommandXboxController(1);
  }

  public static final class FieldConstants {
    public static final double speakerHeight = 2.05; //meters
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
