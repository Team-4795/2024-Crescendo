package frc.robot;

public class StateManager {

    private static State state = State.SPEAKER;
    private static boolean aiming = false;
    private static boolean automate = true;

    public enum State {
        SPEAKER,
        AMP,
        SHUTTLE;
    }

    public static void setState(State desired) {
        state = desired;
    }

    public static State getState(){
        return state;
    }

    public static boolean isAutomate() {
        return automate;
    }

    public static void toggleAutomate() {
        automate = !automate;
    }

    public static boolean isAiming(){
        return aiming;
    }

    public static void setAim(boolean aim){
        aiming = aim;
    }
}
