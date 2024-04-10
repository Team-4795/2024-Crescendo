package frc.robot.util;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoCommands;

public class AutoGenerator {
    public enum ScoringSide {
        SOURCE,
        AMP;
    }

    public enum Range{
        FAR,
        CLOSE;
    }

    LoggedDashboardChooser<ScoringSide> scoreSide = new LoggedDashboardChooser<>("Scoring Side");
    LoggedDashboardChooser<Integer> firstNote = new LoggedDashboardChooser<>("First Note");
    LoggedDashboardChooser<Integer> secondNote = new LoggedDashboardChooser<>("Second Note");
    
    LoggedDashboardChooser<Range> firstNoteRange = new LoggedDashboardChooser<>("First Note Range");
    LoggedDashboardChooser<Range> secondNoteRange = new LoggedDashboardChooser<>("Second Note Range");


    public AutoGenerator(){
        for(int i = 4; i < 8; i++){
            firstNote.addOption("Note " + i, Integer.valueOf(i));
            secondNote.addOption("Note " + i, Integer.valueOf(i));
        }

        for(Range range : Range.values()){
            firstNoteRange.addOption(range.toString(), range);
            secondNoteRange.addOption(range.toString(), range);
        }

        for(ScoringSide side : ScoringSide.values()){
            scoreSide.addOption(side.toString(), side);
        }
    }

    public Command getGeneratedAuto(){
        return Commands.sequence(
            Commands.parallel(
                AutoCommands.rotateToSpeaker(),
                AutoCommands.aimSpeakerDynamic(true, 5000)
            ),
            AutoCommands.score(),

            AutoCommands.getCompleteSequence(firstNote.get(), scoreSide.get(), firstNoteRange.get()),
            AutoCommands.getCompleteSequence(secondNote.get(), scoreSide.get(), secondNoteRange.get())
        );
    }
}
