package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

  private final AddressableLED leds;
  private final AddressableLEDBuffer ledBuffer;

  public LEDs() {
    leds = new AddressableLED(9);

    ledBuffer = new AddressableLEDBuffer(57);
    leds.setLength(ledBuffer.getLength());

    leds.setData(ledBuffer);
    leds.start();
  }
}
