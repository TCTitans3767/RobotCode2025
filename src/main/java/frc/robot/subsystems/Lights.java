package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase{
    private final AddressableLED leds;
    private final AddressableLEDBuffer ledBuffer;

    public Lights() {
        leds = new AddressableLED(9);
        ledBuffer = new AddressableLEDBuffer(9);
        leds.setLength(ledBuffer.getLength());
        leds.setData(ledBuffer);
        setLights(colors.GREEN);
    }

    public enum colors {
        RED,
        GREEN,
        BLUE,
        WHITE,
        OFF
    }

    public void setLights(colors color) {
        switch (color) {
            case RED:
                for (var i = 0; i < ledBuffer.getLength(); i++) {
                    ledBuffer.setRGB(i, 255, 0, 0);
                }
                break;

            case GREEN:
                for (var i = 0; i < ledBuffer.getLength(); i++) {
                    ledBuffer.setRGB(i, 0, 255, 0);
                }
                break;

            case BLUE:
                for (var i = 0; i < ledBuffer.getLength(); i++) {
                    ledBuffer.setRGB(i, 0, 0, 255);
                }
                break;
            
            case WHITE:
                for (var i = 0; i < ledBuffer.getLength(); i++) {
                    ledBuffer.setRGB(i, 255, 255, 255);
                }
                break;

            case OFF:
                for (var i = 0; i < ledBuffer.getLength(); i++) {
                    ledBuffer.setRGB(i, 0, 0, 0);
                }
                break;

            default:
                break;
        }
        leds.setData(ledBuffer);
    }
    
}
