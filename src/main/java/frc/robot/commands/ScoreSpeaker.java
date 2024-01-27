package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;

public class ScoreSpeaker extends Command{

    public final double speakerHeight = 1.98;
    public final double pivotHeight = 0.2794;
    public double distanceToSpeaker = 1;
    public double angleCalc = 0.0;
    


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
}
