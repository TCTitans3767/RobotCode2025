package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader.IndexedColorIterator;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotLights extends SubsystemBase {

    AddressableLED backLED;
    AddressableLEDBuffer backBuffer;
    LEDPattern greenPattern = LEDPattern.solid(Color.kGreen);

    public RobotLights() {
        backLED = new AddressableLED(0);
        backBuffer = new AddressableLEDBuffer(60);
        backLED.setLength(60);
        greenPattern.applyTo(backBuffer);
        backLED.setData(backBuffer);
        backLED.start();
    }

    @Override
    public void periodic() {
    }

    public void setBackLEDColor(Color color) {
        for (int i = 0; i < backBuffer.getLength(); i++) {
            backBuffer.setLED(i, color);
        }
        backLED.setData(backBuffer);
    }
    
}
