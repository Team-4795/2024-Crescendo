package frc.robot.autoPaths;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoPath {
    public static Command load() {
        return null;
    }
    public static boolean next = false;
    public static boolean next2 = false;

    public static boolean next(){
        return next;
    }

    public static void setNext(boolean on){
        next = on;
    }

    public static boolean next2(){
        return next2;
    }

    public static void setNext2(boolean on){
        next2 = on;
    }
}