// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MAXSwerve;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.MAXSwerve.DriveConstants.AutoConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Drive extends SubsystemBase {
    
    private final Module[] modules = new Module[4];

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
    
      // Odometry class for tracking robot pose
    private SwerveDrivePoseEstimator m_poseEstimator;
    static final Lock odometryLock = new ReentrantLock();
    private EstimatedRobotPose visionPose = new EstimatedRobotPose(new Pose3d(), m_currentRotation, null, null);
    private Rotation2d gyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

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
        modules[0] = new Module(fl, 0);
        modules[1] = new Module(fr, 1);
        modules[2] = new Module(bl, 2);
        modules[3] = new Module(br, 3);

        for(Module module : modules){
            module.updateInputs();
        }

        vision = Vision.getInstance();

        OdometryThread.getInstance().start();

          // Odometry class for tracking robot pose
        m_poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
                gyroInputs.yaw,
                new SwerveModulePosition[] {
                        modules[0].getPosition(),
                        modules[1].getPosition(),
                        modules[2].getPosition(),
                        modules[3].getPosition()
                }, new Pose2d(0,0, new Rotation2d(0)));

        this.zeroHeading();

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
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
            this.drive(
                -MathUtil.applyDeadband(OIConstants.driverController.getLeftY(), OIConstants.kAxisDeadband),
                -MathUtil.applyDeadband(OIConstants.driverController.getLeftX(), OIConstants.kAxisDeadband),
                -MathUtil.applyDeadband(OIConstants.driverController.getRightX(), OIConstants.kAxisDeadband),
                fieldRelative, true);
        }, this));
    }

    @Override
    public void periodic() {
        odometryLock.lock();
        gyro.updateInputs(gyroInputs);
        Logger.processInputs("Gyro", gyroInputs);
        for(Module module : modules){
            module.updateInputs();
        }
        odometryLock.unlock();

        double[] sampleTimestamps = modules[0].getOdometryTimestamps();
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                    modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            if(gyroInputs.connected){
                gyroRotation = gyroInputs.odometryYaw[i];
            } else {
                var twist = DriveConstants.kDriveKinematics.toTwist2d(moduleDeltas);
                gyroRotation = gyroRotation.plus(Rotation2d.fromRadians(twist.dtheta));
                gyro.addOffset(Rotation2d.fromRadians(twist.dtheta));
                pose = pose.exp(twist);
            }

            m_poseEstimator.updateWithTime(sampleTimestamps[i], gyroRotation, modulePositions);
        }
     
        if (Constants.currentMode == Mode.REAL && Constants.hasVision) {
            vision.getBarbaryFigPose(m_poseEstimator.getEstimatedPosition()).ifPresent(pose -> {
                m_poseEstimator.addVisionMeasurement(
                    pose.estimatedPose.toPose2d(), 
                    pose.timestampSeconds,
                    VecBuilder.fill(1,1,Units.degreesToRadians(20)));
                    Logger.recordOutput("Vision/Barbary Fig Pose", pose.estimatedPose.toPose2d());
            });

            vision.getSaguaroPose(m_poseEstimator.getEstimatedPosition()).ifPresent(pose -> {
                m_poseEstimator.addVisionMeasurement(
                    pose.estimatedPose.toPose2d(), 
                    pose.timestampSeconds,
                    VecBuilder.fill(1,1,Units.degreesToRadians(20)));
                Logger.recordOutput("Vision/Saguaro Pose", pose.estimatedPose.toPose2d());
            }); 
        }

        Logger.recordOutput("Estimated Pose", getPose());
        Logger.recordOutput("Vision pose", visionPose.estimatedPose);
        Logger.recordOutput("Vision/Distance to speaker", vision.getDistancetoSpeaker(getPose()));
        Logger.recordOutput("Odometry", getPose());
        Logger.recordOutput("Simulated Pose", pose);
        Logger.recordOutput("Swerve/SwerveStates", this.getModuleStates());
        Logger.recordOutput("Swerve/OptimizedStates", this.getOptimizedStates());
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
                getModulePositions(),
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
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

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

        Rotation2d fieldRelativeRotation;
        switch(Constants.currentMode){
            case REAL:
                fieldRelativeRotation = gyroInputs.yaw;
                break;
            case SIM:
                fieldRelativeRotation = pose.getRotation();
                break;
            default:
                fieldRelativeRotation = new Rotation2d();
                break;

        }

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                fieldRelativeRotation)
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        
        for(int i = 0; i < modules.length; i++){
            modules[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        modules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        modules[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        modules[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        modules[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] currentState = new SwerveModuleState[modules.length];
        for(int i = 0; i < modules.length; i++){
            currentState[i] = modules[i].getState();
        }
        return currentState;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] position = new SwerveModulePosition[modules.length];
        for(int i = 0; i < modules.length; i++){
            position[i] = modules[i].getPosition();
        }
        return position;
    }

    public SwerveModuleState[] getOptimizedStates() {
        SwerveModuleState[] currentState = new SwerveModuleState[modules.length];
        for(int i = 0; i < modules.length; i++){
            currentState[i] = modules[i].getOptimizedState();
        }
        return currentState;
    }

    public Translation2d getTranslationVelocity(){
        ChassisSpeeds speed = DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
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
        for(int i = 0; i < modules.length; i++){
            modules[i].setDesiredState(desiredStates[i]);
        }
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        for(Module module : modules){
            module.resetEncoders();
        }
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
        pose.rotateBy(pose.getRotation().times(-1)); //This may not work
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

    //between -180 and 180
    public double getWrappedHeading(){
        double rotation = gyroInputs.yaw.getDegrees() % 360;
        if(rotation < -180) {
            rotation += 360;
        } else if (rotation > 180){
            rotation -= 360;
        }
        return rotation;
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

    /* private void driveRobotRelative(ChassisSpeeds speeds) {
        // This takes the velocities and converts them into precentages (-1 to 1)
        drive(speeds.vxMetersPerSecond / AutoConstants.kMaxSpeedMetersPerSecond,
                speeds.vyMetersPerSecond / AutoConstants.kMaxSpeedMetersPerSecond,
                speeds.omegaRadiansPerSecond / AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                false,
                true);
       
        } */

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(moduleStates);
    }
    
    public void runVelocity(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyroInputs.yaw)
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, DriveConstants.kMaxSpeedMetersPerSecond);
        for(int i = 0; i < modules.length; i++){
            modules[i].setDesiredState(swerveStates[i]);
        }
    }
}
