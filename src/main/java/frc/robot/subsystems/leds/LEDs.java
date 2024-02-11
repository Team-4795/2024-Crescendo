package frc.robot.subsystems.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private final int LED_LENGTH = 0;
    private final int PORT = 0;

    private AddressableLED led;
    private AddressableLEDBuffer buffer;
    private String[] led1 = new String[1];


    public LEDs() {
        led = new AddressableLED(PORT);
        buffer = new AddressableLEDBuffer(LED_LENGTH);
        led.setLength(buffer.getLength());
        
    }

    private void setColor(int a0, int a1, int a2, boolean colorModel, int start, int end) /* false: RGB; true: HSV */ {
        start = MathUtil.clamp(start, 0, LED_LENGTH);
        end = MathUtil.clamp(end, start, LED_LENGTH);

        for (int i = start; i <= end; i++) {
            if (colorModel) {buffer.setHSV(i, a0, a1, a2);}
            else {buffer.setRGB(i, a0, a1, a2);}
        }

        led.setData(buffer);
        led.start();

    }

    public void setRGB(int r, int g, int b, int led) {
        setColor(r, g, b, false, led, led);
    }
    
    public void setStrip(int r, int g, int b) {
        setColor(r, g, b, false, 0, LED_LENGTH);
    }

    public void setTopColor(int r, int g, int b) {
        setColor(r, g, b, false, LED_LENGTH/2, LED_LENGTH);
    }

    public void setBottomColor(int r, int g, int b) {
        setColor(r, g, b, false, 0, LED_LENGTH/2);
    }

}
