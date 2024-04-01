// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MAXSwerve;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.Tolerances;
import frc.robot.commands.AutoAlignAmp;
import frc.robot.subsystems.MAXSwerve.DriveConstants.AutoConstants;
import frc.robot.subsystems.vision.AprilTagVision.Vision;
import frc.robot.util.ChassisSpeedsUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SwerveUtils;
import frc.robot.util.Util.AllianceFlipUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Drive extends SubsystemBase {
    private final Module m_frontLeft;
    private final Module m_frontRight;
    private final Module m_rearLeft;
    private final Module m_rearRight;

    // The gyro sensor
    private GyroIO gyro;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private boolean fieldRelative = true;

    // Slew rate filter variables for controlling lateral acceleration
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;
    
    //PID controllers for auto-alignment
    private static final LoggedTunableNumber linearkP = new LoggedTunableNumber("AutoAlign/drivekP", 1.0);
    private static final LoggedTunableNumber linearkD = new LoggedTunableNumber("AutoAlign/drivekD", 0.0);
    private static final LoggedTunableNumber thetakP = new LoggedTunableNumber("AutoAlign/thetakP", 1.0);
    private static final LoggedTunableNumber thetakD = new LoggedTunableNumber("AutoAlign/thetakD", 0.5);
    private static final LoggedTunableNumber linearTolerance = new LoggedTunableNumber(
            "AutoAlign/translationalTolerance", 0.1);
    private static final LoggedTunableNumber thetaTolerance = new LoggedTunableNumber(
            "AutoAlign/rotationalTolerance", Units.degreesToRadians(5.0));
    private ProfiledPIDController translationController;
    private ProfiledPIDController rotationController;

      // Odometry class for tracking robot pose
    SwerveDrivePoseEstimator m_poseEstimator;

    private Transform2d lastGyroVel = new Transform2d();
    private Transform2d gyroVel = new Transform2d();
    @AutoLogOutput
    private Pose2d gyroPos = new Pose2d();
    private int ticks = 0; 

    private Vision vision;

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry;
    private Pose2d pose = new Pose2d();
    private static Drive instance;

    public static Drive getInstance(){
        return instance;
    }

    public static Drive initialize(GyroIO gyro, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br){
        if(instance == null){
            instance = new Drive(gyro, fl, fr, bl, br);
        }
        return instance;
    }


    /** Creates a new DriveSubsystem. */
    private Drive(GyroIO gyro, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
        this.gyro = gyro;
        m_frontLeft = new Module(fl, 0);
        m_frontRight = new Module(fr, 1);
        m_rearLeft = new Module(bl, 2);
        m_rearRight = new Module(br, 3);

        m_frontLeft.updateInputs();
        m_frontRight.updateInputs();
        m_rearLeft.updateInputs();
        m_rearRight.updateInputs();
        vision = Vision.getInstance();

          // Odometry class for tracking robot pose
        m_poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
                gyroInputs.yaw,
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                }, new Pose2d(0,0, Rotation2d.fromDegrees(0)));

        this.zeroHeading();

        translationController = new ProfiledPIDController(linearkP.get(), 0, linearkD.get(), new Constraints(3, 5));
        rotationController = new ProfiledPIDController(thetakP.get(), 0, thetakD.get(), new Constraints(5.5, 6.5));
        translationController.setTolerance(linearTolerance.get());
        rotationController.setTolerance(thetaTolerance.get());

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(AutoConstants.kPDrivingController,0,0), // Translation PID constants
                        new PIDConstants(AutoConstants.kPThetaController,0,0), // Rotation PID constants
                        4.5, // Max module speed, in m/s ***MIGHT CHANGE***
                        DriveConstants.kTrackRadius, // Drive base radius in meters. Distance from robot center to
                                                     // furthest module. ***MIGHT CHANGE***
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red,
                this // Reference to this subsystem to set requirements
        );

        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                }); // Adds a way for PathPlanner to log what poses it's trying to get the robot to
                    // go to
        
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                }); // Adds a way for PathPlanner to log what pose it's currently trying to go to

        setDefaultCommand(Commands.run(() -> {
            double xSpeed = -OIConstants.driverController.getLeftY();
            double ySpeed = -OIConstants.driverController.getLeftX();
            double rotSpeed = -OIConstants.driverController.getRightX();

            double speed = MathUtil.applyDeadband(Math.hypot(xSpeed, ySpeed), OIConstants.kAxisDeadband);
            Rotation2d direction = new Rotation2d(xSpeed, ySpeed);
            double omega = MathUtil.applyDeadband(rotSpeed, OIConstants.kAxisDeadband);

            speed = speed * speed;
            omega = Math.copySign(omega * omega, omega);

            Translation2d velocity = new Pose2d(new Translation2d(), direction)
                .transformBy(new Transform2d(speed, 0, new Rotation2d()))
                .getTranslation();

            this.drive(
                velocity.getX(),
                velocity.getY(),
                omega,
                fieldRelative, 
                true);
        }, this));
    }

    private void updateGyro() {
        lastGyroVel = new Transform2d(getTranslationVelocity(), new Rotation2d());
        gyroVel = lastGyroVel;
        gyroPos = getPose();
    }

    @Override
    public void periodic() {
        gyro.updateInputs(gyroInputs);
        Logger.processInputs("Gyro", gyroInputs);
        m_frontLeft.updateInputs();
        m_frontRight.updateInputs();
        m_rearLeft.updateInputs();
        m_rearRight.updateInputs();

        ticks += 1;
        if (ticks % 200 == 0) {
            updateGyro();
        } else {
            // var accel = new Translation3d(gyroInputs.accel[0], gyroInputs.accel[1], gyroInputs.accel[2])
            //     .plus(new Translation3d(0, 0, -9.80665).rotateBy(new Rotation3d(gyroInputs.roll, gyroInputs.pitch, gyroInputs.yaw.getRotations())));
            // Logger.recordOutput("Gyro Accel", accel);
            gyroVel = new Transform2d(new Translation2d(gyroVel.getX() + gyroInputs.accel[0] * 0.02, gyroVel.getY() + (gyroInputs.accel[1] + 0.23) * 0.02), new Rotation2d());
            gyroPos = new Pose2d(gyroPos.getTranslation(), getRotationHeading()).transformBy(gyroVel.times(0.02));
        }

        m_poseEstimator.update(
                gyroInputs.yaw,
                new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
                });

        Vision.getInstance().setReferencePose(m_poseEstimator.getEstimatedPosition());
        
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> translationController.setPID(linearkP.get(), 0, linearkD.get()),
            linearkP, linearkD);
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> rotationController.setPID(thetakP.get(), 0, thetakD.get()),
            thetakP, thetakD);
        LoggedTunableNumber.ifChanged(
            hashCode(), () -> translationController.setTolerance(linearTolerance.get()), linearTolerance);
        LoggedTunableNumber.ifChanged(
            hashCode(), () -> rotationController.setTolerance(thetaTolerance.get()), thetaTolerance);

        Logger.recordOutput("Estimated Pose", getPose());
        Logger.recordOutput("Vision/Distance to speaker", vision.getDistancetoSpeaker(getPose()));
        
        // Read wheel deltas from each module
        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
        wheelDeltas[0] = m_frontLeft.getPositionDelta();
        wheelDeltas[1] = m_frontRight.getPositionDelta();
        wheelDeltas[2] = m_rearLeft.getPositionDelta();
        wheelDeltas[3] = m_rearRight.getPositionDelta();

        // The twist represents the motion of the robot since the last
        // sample in x, y, and theta based only on the modules, without
        // the gyro.
        var twist = DriveConstants.kDriveKinematics.toTwist2d(wheelDeltas);

        if (!gyroInputs.connected) {
            gyro.addOffset(Rotation2d.fromRadians(twist.dtheta));
        }
        // Apply the twist (change since last sample) to the current pose
        pose = pose.exp(twist);

        Logger.recordOutput("Odometry", getPose());
        Logger.recordOutput("Simulated Pose", pose);
        Logger.recordOutput("Swerve/SwerveStates", this.getModuleStates());
        Logger.recordOutput("Swerve/OptimizedStates", this.getOptimizedStates());
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stddevs) {
        m_poseEstimator.addVisionMeasurement(pose, timestamp, stddevs);
    }

    /* Returns if a shot at an angle will go into the speaker */
    public boolean atSpeakerAngle() {
        Rotation2d min = AllianceFlipUtil.apply(FieldConstants.BLUE_SPEAKER).getTranslation()
            .plus(new Translation2d(0, Tolerances.speakerWidth))
            .minus(getPose().getTranslation()).getAngle();

        Rotation2d max = AllianceFlipUtil.apply(FieldConstants.BLUE_SPEAKER).getTranslation()
            .plus(new Translation2d(0, -Tolerances.speakerWidth))
            .minus(getPose().getTranslation()).getAngle();

        return between(getPose().getRotation().rotateBy(new Rotation2d(Math.PI)), min, max);
    }

    public double wrapDeg(double angle) {
        return MathUtil.inputModulus(angle, 0, 360);
    }

    public boolean between(Rotation2d x, Rotation2d min, Rotation2d max) {
        if (max.minus(min).getSin() < 0.0) {
            return between(x, max, min);
        }

        return wrapDeg(x.getDegrees() - min.getDegrees()) <= wrapDeg(max.getDegrees() - min.getDegrees());
    }

    public boolean slowMoving() {
        return getTranslationVelocity().getNorm() < Tolerances.driveVelocity
            && getTurnRate() < Tolerances.turningSpeed;
    }   

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(
                gyroInputs.yaw,
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                pose);

        this.pose = pose;
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {
            // Convert XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.hypot(xSpeed, ySpeed);

            // Calculate the direction slew rate based on an estimate of the lateral
            // acceleration
            double directionSlewRate;
            if (m_currentTranslationMag != 0.0) {
                directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
            } else {
                directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - m_prevTime;
            double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
            if (angleDif < 0.45 * Math.PI) {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85 * Math.PI) {
                if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality
                                                      // checking
                    // keep currentTranslationDir unchanged
                    m_currentTranslationMag = m_magLimiter.calculate(0.0);
                } else {
                    m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                }
            } else {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(0.0);
            }
            m_prevTime = currentTime;

            xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
            ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
            m_currentRotation = m_rotLimiter.calculate(rot);

        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            m_currentRotation = rot;
        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

        Logger.recordOutput("Swerve/XspeedCommanded", xSpeedDelivered);
        Logger.recordOutput("Swerve/YspeedCommanded", ySpeedDelivered);

        // Rotation2d fieldRelativeRotation;
        // switch(Constants.currentMode){
        //     case REAL:
        //         fieldRelativeRotation = gyroInputs.yaw;
        //         break;
        //     case SIM:
        //         fieldRelativeRotation = pose.getRotation();
        //         break;
        //     default:
        //         fieldRelativeRotation = new Rotation2d();
        //         break;
        // }

        ChassisSpeeds speeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
        runVelocity(speeds, fieldRelative);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] currentState = new SwerveModuleState[4];
        currentState[0] = m_frontLeft.getState();
        currentState[1] = m_frontRight.getState();
        currentState[2] = m_rearLeft.getState();
        currentState[3] = m_rearRight.getState();
        return currentState;
    }

    public SwerveModuleState[] getOptimizedStates() {
        SwerveModuleState[] currentState = new SwerveModuleState[4];
        currentState[0] = m_frontLeft.getOptimizedState();
        currentState[1] = m_frontRight.getOptimizedState();
        currentState[2] = m_rearLeft.getOptimizedState();
        currentState[3] = m_rearRight.getOptimizedState();
        return currentState;
    }

    public Translation2d getTranslationVelocity(){
        SwerveModuleState[] currentState = new SwerveModuleState[4];
        currentState[0] = m_frontLeft.getState();
        currentState[1] = m_frontRight.getState();
        currentState[2] = m_rearLeft.getState();
        currentState[3] = m_rearRight.getState();
        ChassisSpeeds speed = DriveConstants.kDriveKinematics.toChassisSpeeds(currentState);
        return new Translation2d(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
    }

    public Translation2d getFieldRelativeTranslationVelocity(){
        SwerveModuleState[] currentState = new SwerveModuleState[4];
        currentState[0] = m_frontLeft.getFieldRelativeState(gyroInputs.yaw);
        currentState[1] = m_frontRight.getFieldRelativeState(gyroInputs.yaw);
        currentState[2] = m_rearLeft.getFieldRelativeState(gyroInputs.yaw);
        currentState[3] = m_rearRight.getFieldRelativeState(gyroInputs.yaw);
        ChassisSpeeds speed = DriveConstants.kDriveKinematics.toChassisSpeeds(currentState);
        return new Translation2d(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    public Command AutoAlignAmp(){
        return new AutoAlignAmp(translationController, rotationController);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        if (Constants.currentMode == Mode.SIM) {
            m_poseEstimator.resetPosition(new Rotation2d(), 
                new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
                }, getPose());
        }
        
        gyro.reset();
    }

    public void setFieldRelative(boolean fieldRelative){
        this.fieldRelative = fieldRelative;
    }

    public double getHeading() {
        return gyroInputs.yaw.getDegrees();
    }

    public Rotation2d getRotationHeading() {
        return gyroInputs.yaw;
    }

    public double getWrappedHeading(){
        return MathUtil.angleModulus(gyroInputs.yaw.getRadians());
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in radians per second
     */
    public double getTurnRate() {
        return gyroInputs.yawVelocity;
    }

    private ChassisSpeeds getRobotRelativeSpeeds() {
        // Uses forward kinematics to calculate the robot's speed given the states of
        // the swerve modules.
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

     /*private void driveRobotRelative(ChassisSpeeds speeds) {
        // This takes the velocities and converts them into precentages (-1 to 1)
        drive(speeds.vxMetersPerSecond / AutoConstants.kMaxSpeedMetersPerSecond,
                speeds.vyMetersPerSecond / AutoConstants.kMaxSpeedMetersPerSecond,
                speeds.omegaRadiansPerSecond / AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                false,
                true);
       
  }  */

    public void driveRobotRelative(ChassisSpeeds speeds)
    {
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(moduleStates);
    }
    
    public void runVelocity(ChassisSpeeds speeds, boolean fieldRelative) {
        speeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyroInputs.yaw)
            : speeds;

        speeds = ChassisSpeedsUtil.correctForDynamics(speeds, 0.02, DriveConstants.driftRate);
        SwerveModuleState[] swerveStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(swerveStates);
    }
}
