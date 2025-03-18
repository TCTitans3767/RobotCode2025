package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader.IndexedColorIterator;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotLights extends SubsystemBase {

    AddressableLED backLED;
    // AddressableLED frontLED;
    AddressableLEDBuffer backBuffer;
    // AddressableLEDBuffer frontBuffer;
    LEDPattern greenPattern = LEDPattern.solid(Color.kBlue);

    public RobotLights() {
        // frontLED = new AddressableLED(9);
        backLED = new AddressableLED(8);
        backBuffer = new AddressableLEDBuffer(60);
        // frontBuffer = new AddressableLEDBuffer(60);
        backLED.setLength(60);
        // frontLED.setLength(60);
        greenPattern.applyTo(backBuffer);
        // greenPattern.applyTo(frontBuffer);
        backLED.setData(backBuffer);
        // frontLED.setData(frontBuffer);
        backLED.start();
        // frontLED.start();
    }

    @Override
    public void periodic() {
    }

    public void setBackLEDColor(LEDPattern pattern) {
        pattern.applyTo(backBuffer);
        backLED.setData(backBuffer);
    }
    
}
