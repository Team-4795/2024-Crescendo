package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.vision.Vision;

public class AlignToGamepiece extends Command {
    private Drive drive = Drive.getInstance();
    private Vision vision = Vision.getInstance();

    private PIDController rotationPID = new PIDController(0.35, 0, 0); 
    
    public AlignToGamepiece() {
        addRequirements(drive);
        if(Constants.hasVision){
            vision = Vision.getInstance();
            addRequirements(vision);
        }
        rotationPID.enableContinuousInput(-180, 180);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute() {
       double lifecamYaw = vision.getLifecamYaw();
       
       double output = MathUtil.clamp(rotationPID.calculate(Units.degreesToRadians(lifecamYaw), 0), -1, 1);
       
        drive.drive(
            -MathUtil.applyDeadband(OIConstants.driverController.getLeftY(), OIConstants.kAxisDeadband),
            0,
            output,
            false, true);    
        
        Logger.recordOutput("Vision/Note Yaw", lifecamYaw);
        Logger.recordOutput("Vision/Note PID", output);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
