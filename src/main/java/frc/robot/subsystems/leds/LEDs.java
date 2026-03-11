package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

  private final AddressableLED leds;
  private final AddressableLEDBuffer ledBuffer;
  private AddressableLEDBufferView frontSection;
  private AddressableLEDBufferView backSection;

  public LEDs() {
    leds = new AddressableLED(9);

    ledBuffer = new AddressableLEDBuffer(57);
    this.frontSection = ledBuffer.createView(0, 33);
    this.backSection = ledBuffer.createView(34, 56);
    leds.setLength(ledBuffer.getLength());

    leds.setData(ledBuffer);
    leds.start();

  }
  public Command runPattern(LEDPattern pattern) {
    LEDPattern pattern_dim = pattern.atBrightness((Percent.of(75)));
    return run(() -> {
      pattern_dim.applyTo(frontSection); // set pixel data for both views
      pattern_dim.applyTo(backSection);
      leds.setData(ledBuffer); // send pixel data to the LED strips
    }).ignoringDisable(true);

  }

  public Command showFlywheelAtSpeed() {
    return runPattern(LEDPattern.solid(Color.kGreen));
  }

  public Command showFeed() {
    return runPattern(LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kYellow, Color.kGreen)
        .scrollAtRelativeSpeed(Percent.per(Second).of(25)));
  }

  public Command showHopperIntake() {
    return runPattern(LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kOrange, Color.kYellow)
        .scrollAtRelativeSpeed(Percent.per(Second).of(25)));
  }

  public Command showExtendAndGrab() {
    return runPattern(LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kOrange)
        .scrollAtRelativeSpeed(Percent.per(Second).of(25)));
  }
}
