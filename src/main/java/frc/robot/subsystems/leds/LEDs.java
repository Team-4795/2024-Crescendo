package frc.robot.subsystems.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private final int LED_LENGTH = 27;
    private final int PORT = 0;

    private AddressableLED led;
    private AddressableLEDBuffer buffer;

    // private String[] led1 = new String[1];

    private int frame;

    private boolean teamColorsAnimation = false;

    private boolean yellow = false;
    private boolean pathfinding = false;
    private boolean revving = false;
    private boolean canShoot = false;

    private static LEDs instance;

    public static LEDs getInstance() {
        if (instance == null) {
            instance = new LEDs();
        }

        return instance;
    }

    private LEDs() {
        led = new AddressableLED(PORT);
        buffer = new AddressableLEDBuffer(LED_LENGTH);
        led.setLength(buffer.getLength());

        setDefaultCommand(run(() -> {
            if (canShoot) {
                setColor(Color.kMagenta);
            } else if (revving) {
                setColor(Color.kFirstBlue);  
            } else if (pathfinding) {
                setColor(Color.kOrange);
            } else if (yellow) {
                setColor(Color.kYellow);
            } else {
                setTeamColors();
            }
        }).ignoringDisable(true));

        setTeamColors();
    };

    // Purple 143 139 189
    // Blue 1 195 203

    private void setTeamColors() {
        setColorNoOutput(174, 19, 186, false, 1, 8);
        setColorNoOutput(1, 195, 203, false, 8, 14);

        // Flip for other led strip
        setColorNoOutput(1, 195, 203, false, 14, 20);
        setColorNoOutput(174, 19, 186, false, 20, 27);

        setOutput();
    }

    public Command intook() {
        return runOnce(() -> setColor(Color.kGreen)).andThen(Commands.waitSeconds(3));
    }

    public Command pathfinding() {
        return Commands.startEnd(() -> pathfinding = true, () -> pathfinding = false);
    }

    public Command revving() {
        return Commands.startEnd(() -> revving = true, () -> revving = false);
    }

    public Command canShoot() {
        return Commands.startEnd(() -> canShoot = true, () -> canShoot = false);
    }

    public void toggleYellow() {
        yellow = !yellow;
    }

    public void setColor(Color color) {
        setStripRGB((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
    }

    private void setColorNoOutput(int a0, int a1, int a2, boolean colorModel, int start, int end) /* false: RGB; true: HSV */ {
        start = MathUtil.clamp(start, 0, LED_LENGTH);
        end = MathUtil.clamp(end, start, LED_LENGTH);

        for (int i = start; i < end; i++) {
            if (colorModel) buffer.setHSV(i, a0, a1, a2);
            else buffer.setRGB(i, a0, a1, a2);
        }
    }

    private void setColor(int a0, int a1, int a2, boolean colorModel, int start, int end) /* false: RGB; true: HSV */ {
        setColorNoOutput(a0, a1, a2, colorModel, start, end);
        setOutput();
    }

    public void setLEDRGB(int r, int g, int b, int led) {
        setColor(r, g, b, false, led, led);
    }

    public void setPartRGB(int r, int g, int b, int start, int end) {
        setColor(r, g, b, false, start, end);
    }
    
    public void setStripRGB(int r, int g, int b) {
        setColor(r, g, b, false, 0, LED_LENGTH);
    }

    public void setTopColorRGB(int r, int g, int b) {
        setColor(r, g, b, false, LED_LENGTH/2, LED_LENGTH);
    }

    public void setBottomColorRGB(int r, int g, int b) {
        setColor(r, g, b, false, 0, LED_LENGTH/2);
    }

    public void setPartHSV(int h, int s, int v, int start, int end) {
        setColor(h, s, v, true, start, end);
    }

    public void setStripHSV(int h, int s, int v) {
        setColor(h, s, v, true, 0, LED_LENGTH);
    }

    public void setTopColorHSV(int h, int s, int v) {
        setColor(h, s, v, true, LED_LENGTH/2, LED_LENGTH);
    }

    public void setBottomColorHSV(int h, int s, int v) {
        setColor(h, s, v, true, 0, LED_LENGTH/2);
    }

    // public void setStripAlliance() {
    //     if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
    //         setStripRGB(255, 0, 0);
    //     }
    //     else {
    //         setStripRGB(0, 0, 255);
    //     }
    // }
    
    public void toggleTeamColorsAnimation() {
        if (teamColorsAnimation) {
            teamColorsAnimation = false;
        }
        else {
            teamColorsAnimation = true;
        }
    }

    private void updateTeamColor() {
        if ((frame/10) % 2 == 0) {
            setStripRGB(143, 139, 189);
        }
        else {
            setStripRGB(1, 195, 203);
        }
    }

    public void setHSVIndex(int index, int h, int s, int v) {
        buffer.setHSV(index, h, s, v);
    }

    public int getLength() {
        return buffer.getLength();
    }

    public void setOutput() {
        led.setData(buffer);
        led.start();
    }

    // @Override
    // public void periodic() {
    //     // updates LEDs to show state of intake
    //     if (Indexer.getInstance().isStoring()) {
    //         setStripRGB(0, 255, 0);
    //     } 
    //     else {
    //         setStripRGB(255, 0, 0);
    //     }
    //     if (teamColorsAnimation) {
    //         updateTeamColor();
    //     }
    // }

}
//143 139 189
//1 195 203

//142, 46, 14, ratio is 71:23:7