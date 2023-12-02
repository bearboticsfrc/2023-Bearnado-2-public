package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.LedConstants;

public class LedSubsystem extends SubsystemBase {

  private static final CANdle candle = new CANdle(LedConstants.PORT);

  public static final Color blue = new Color(0, 0, 255);
  public static final Color red = new Color(255, 0, 0);
  public static final Color purple = new Color(125, 18, 255);
  public static final Color yellow = new Color(280, 130, 0);
  public static final Color black = new Color(0, 0, 0);

  public LedSubsystem() {
    CANdleConfiguration candleConfiguration = new CANdleConfiguration();
    candleConfiguration.statusLedOffWhenActive = false;
    candleConfiguration.disableWhenLOS = false;
    candleConfiguration.stripType = LEDStripType.RGB;
    candleConfiguration.brightnessScalar = 1.0;
    candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(candleConfiguration, 100);
  }

  public void setBrightness(double percent) {
    candle.configBrightnessScalar(percent, 100);
  }

  public Command defaultCommand() {
    return runOnce(
        () -> {
          LEDSegment.CANdle.setColor(blue);
          LEDSegment.MainStrip.setLarsonAnimation(blue, .6);
        });
  }

  public void runDefaultCommand(boolean isRed) {
    Color color = (isRed ? red : blue);
    LEDSegment.CANdle.setColor(color);
    LEDSegment.MainStrip.setLarsonAnimation(color, .6);
  }

  public void setFastLarsonAnimation(boolean isRed) {
    Color color = (isRed ? red : blue);
    LEDSegment.MainStrip.setLarsonAnimation(color, .9);
  }

  public void setColorPurple() {
    LEDSegment.MainStrip.setColor(purple);
  }

  public void setColorYellow() {
    LEDSegment.MainStrip.setColor(yellow);
  }

  public void turnOff() {
    LEDSegment.MainStrip.disableLEDs();
  }

  public Command clearSegmentCommand(LEDSegment segment) {
    return runOnce(
        () -> {
          segment.clearAnimation();
          segment.disableLEDs();
        });
  }

  public static enum LEDSegment {
    CANdle(0, 8, 0),
    MainStrip(8, 99, 1);

    public final int startIndex;
    public final int segmentSize;
    public final int animationSlot;

    // private enum constructor
    private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
      this.startIndex = startIndex;
      this.segmentSize = segmentSize;
      this.animationSlot = animationSlot;
    }

    public void setColor(Color color) {
      clearAnimation();
      candle.setLEDs(color.red, color.green, color.blue, 0, startIndex, segmentSize);
    }

    private void setAnimation(Animation animation) {
      candle.animate(animation, animationSlot);
    }

    public void clearAnimation() {
      candle.clearAnimation(animationSlot);
    }

    public void disableLEDs() {
      setColor(black);
    }

    public void setLarsonAnimation(Color color, double speed) {
      setAnimation(
          new LarsonAnimation(
              color.red, color.green, color.blue, 0, speed, segmentSize, BounceMode.Front, 7));
    }

    public void setFlowAnimation(Color color, double speed) {
      setAnimation(
          new ColorFlowAnimation(
              color.red,
              color.green,
              color.blue,
              0,
              speed,
              segmentSize,
              Direction.Forward,
              startIndex));
    }

    public void setFadeAnimation(Color color, double speed) {
      setAnimation(
          new SingleFadeAnimation(
              color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
    }

    public void setBandAnimation(Color color, double speed) {
      setAnimation(
          new LarsonAnimation(
              color.red,
              color.green,
              color.blue,
              0,
              speed,
              segmentSize,
              BounceMode.Front,
              3,
              startIndex));
    }

    public void setStrobeAnimation(Color color, double speed) {
      setAnimation(
          new StrobeAnimation(
              color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
    }

    public void setRainbowAnimation(double speed) {
      setAnimation(new RainbowAnimation(1, 0.5, segmentSize, false, startIndex));
    }
  }

  public static class Color {
    public int red;
    public int green;
    public int blue;

    public Color(int red, int green, int blue) {
      this.red = red;
      this.green = green;
      this.blue = blue;
    }
  }
}
