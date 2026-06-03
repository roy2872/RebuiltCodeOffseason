package frc.robot.subsystems.leds;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class Leds extends SubsystemBase {
  public enum ledsStates {
    OFF(LEDPattern.kOff),
    WHITE(LEDPattern.solid(Color.kGhostWhite)),
    AQUA(LEDPattern.solid(Color.kAqua)),
    RED(LEDPattern.solid(Color.kRed)),
    PURPLE(LEDPattern.solid(Color.kPurple)),
    PINK(LEDPattern.solid(Color.kPink)),
    FINISH_SCORE( // blink purple
        LEDPattern.solid(Color.kPurple)
            .blink(Time.ofBaseUnits(.2, BaseUnits.TimeUnit.getBaseUnit()), 
              Time.ofBaseUnits(.1, BaseUnits.TimeUnit.getBaseUnit()))),
    BLUE(LEDPattern.solid(Color.kBlue));

    LEDPattern value;

    ledsStates(LEDPattern value) {
      this.value = value;
    }

    public LEDPattern pattern() {
      return this.value;
    }
  }

  @AutoLogOutput(key = "leds/ledsState")
  private ledsStates currentState = ledsStates.OFF;

  private final AddressableLED addressableLEDs = new AddressableLED(8);
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(150);

  public Leds() {
    addressableLEDs.setLength(ledBuffer.getLength());
    addressableLEDs.setData(ledBuffer);
    addressableLEDs.start();
  }

  @Override
  public void periodic() {
    switch (currentState) {
      case OFF -> ledsStates.OFF.pattern().applyTo(ledBuffer);
      case WHITE -> ledsStates.WHITE.pattern().applyTo(ledBuffer);
      case AQUA -> ledsStates.AQUA.pattern().applyTo(ledBuffer);
      case RED -> ledsStates.RED.pattern().applyTo(ledBuffer);
      case PURPLE -> ledsStates.PURPLE.pattern().applyTo(ledBuffer);
      case FINISH_SCORE -> ledsStates.FINISH_SCORE.pattern().applyTo(ledBuffer);
      case BLUE -> ledsStates.BLUE.pattern().applyTo(ledBuffer);
      case PINK -> ledsStates.PINK.pattern().applyTo(ledBuffer);
      default -> ledsStates.OFF.pattern().applyTo(ledBuffer);
    }
  }

  public void setState(ledsStates state) {
    if (currentState == state) {
      return;
    }
    currentState = state;
  }
}
