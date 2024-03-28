package frc.robot.autoPaths;

public class AutoGamepieces {
    private static boolean[] piecesPresent = new boolean[] {true, true, true, true, true};

    //Notes 4 - 8 per normal naming conventions
    public static boolean isGone(int note){
        return !piecesPresent[note - 4];
    }

    public static void setNoteGone(int note){
        piecesPresent[note - 4] = false;
    }
}