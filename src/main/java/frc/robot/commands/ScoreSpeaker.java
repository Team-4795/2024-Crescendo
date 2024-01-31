package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.vision.Vision;

public class ScoreSpeaker extends Command{

    public final double speakerHeight = 1.98;
    public final double pivotHeight = 0.2794;
    public double distanceToSpeaker = 1;
    public double angleCalc = 0.0;
    private Vision vision = Vision.getInstance();
    private PIDController rotationPID = new PIDController(1, 0, 0); // Change Values


    public ScoreSpeaker(){
        addRequirements(Pivot.getInstance());
    }

    @Override
    public void execute() {
        //called every 20 ms
        distanceToSpeaker = 3; //Call drivesubsystem distance method
        angleCalc = Math.atan((speakerHeight-pivotHeight)/distanceToSpeaker);
        Pivot.getInstance().setGoal(angleCalc);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    public static Command turnTowardsSpeaker(Drive drive){
        return Commands.run(
        () -> {
            double deltaY = vision.getSpeakerPos().getY() - drive.getPose().getY();
            double angle = Math.asin(deltaY / vision.getDistancetoSpeaker(drive.getPose())) * 180/Math.PI;
            Logger.recordOutput("Angle", angle);

            drive.drive(
                -MathUtil.applyDeadband(OIConstants.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(OIConstants.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -rotationPID.calculate(vision.getArducamYaw(), angle), 
                true, true);
        }, drive);
    }
}
