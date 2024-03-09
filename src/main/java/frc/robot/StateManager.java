package frc.robot;

public class StateManager {

    private static State state = State.SPEAKER;

    public enum State {
        SPEAKER,
        AMP;
    }

    public static void setState(State desired) {
        state = desired;
    }

    public static State getState(){
        return state;
    }
}
