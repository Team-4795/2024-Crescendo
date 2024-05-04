package frc.robot.autoPaths;

import org.littletonrobotics.junction.Logger;

public class AutoGamepieces {
    
    private static boolean[] notesDetected = new boolean[] {false, false, false, false, false};
    private static boolean[] piecesPresent = new boolean[] {true, true, true, true, true};

    //Notes 4 - 8 per normal naming conventions
    public static boolean isGone(int note){
        return !piecesPresent[note - 4];
    }

    public static void setNoteGone(int note){
        piecesPresent[note - 4] = false;
    }

    public static void setNote(int note, boolean present){
        notesDetected[note - 4] = present;
    }

    public static void updateNotes(int note){
        piecesPresent[note - 4] = notesDetected[note - 4];
    }

    public static void resetNotes(){
        piecesPresent = new boolean[] {true, true, true, true, true};
        notesDetected = new boolean[] {false, false, false, false, false};
    }

    public static void logNotes(){
        Logger.recordOutput("Auto Gamepieces", piecesPresent);
         Logger.recordOutput("Detected Auto Gamepieces", notesDetected);
    }
}