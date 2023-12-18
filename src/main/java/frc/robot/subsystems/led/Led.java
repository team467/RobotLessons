package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.leds.DoubleLEDStrip;
import frc.lib.leds.LEDManager;
import frc.robot.RobotConstants;

public class Led extends SubsystemBase {
  public DoubleLEDStrip ledStrip;

  // private static final COLORS_467 BATTERY_LOW_COLOR = COLORS_467.Orange;

  private Rainbows rainbowLed = new Rainbows();

  private COLORS_467 color = COLORS_467.Black;

  /*
   * Color blind preferred pallet includes White, Black, Red, Blue, Gold
   */

  public enum COLORS_467 {
    White(0xFF, 0xFF, 0xFF, 0xdc267f00),
    Red(0xFF, 0x00, 0x00, 0x99000000),
    Green(0x00, 0x80, 0x00, 0x33663300),
    Blue(0x00, 0x00, 0xCC, 0x1a339900),
    Yellow(0xFF, 0xB1, 0x0A, 0xe6e69d00),
    Pink(0xDC, 0x26, 0x7F, 0xdc267f00),
    Orange(0xFE, 0x61, 0x00, 0xfe6100),
    Black(0x00, 0x00, 0x00, 0x00000000),
    Gold(0xFF, 0xC2, 0x0A, 0xe6e64d00),
    Purple(0x69, 0x03, 0xA3, 0x8000ff00);

    public final int red;
    public final int green;
    public final int blue;
    public final int shuffleboard;

    COLORS_467(int red, int green, int blue, int shuffleboard) {
      this.red = red;
      this.green = green;
      this.blue = blue;
      this.shuffleboard = shuffleboard;
    }

    public Color getColor() {
      return new Color(red, green, blue);
    }
  }

  public Led(int channel) {
    super();

    ledStrip =
        LEDManager.getInstance().createDoubleStrip(RobotConstants.get().ledCount, false);
    LEDManager.getInstance().init(channel);
    for (int i = 0; i < ledStrip.getSize(); i++) {
      ledStrip.setRGB(i, 0, 0, 0);
    }
    rainbowLed.rainbowTimer.start();
  }

  public void resetTimers() {
    rainbowLed.rainbowTimer.reset();
    purpleTimer.reset();
  }

  @Override
  public void periodic() {
    set(color.getColor());
    ledStrip.update();
  }


  public void set(Color color) {
    setTop(color);
    setBottom(color);
  }

  public void setTop(Color color) {
    for (int i = 0; i < RobotConstants.get().ledCount / 2; i++) {
      ledStrip.setLED(i, color);
    }
  }

  public void setBottom(Color color) {
    for (int i = RobotConstants.get().ledCount / 2;
        i < RobotConstants.get().ledCount;
        i++) {
      ledStrip.setLED(i, color);
    }
  }

  public void set(COLORS_467 color) {
    setTop(color);
    setBottom(color);
  }

  public void setTop(COLORS_467 color) {
    for (int i = RobotConstants.get().ledCount / 2;
        i < RobotConstants.get().ledCount;
        i++) {
      ledStrip.setRGB(i, color.red, color.green, color.blue);
    }
  }

  public void setBottom(COLORS_467 color) {
    for (int i = 0; i < RobotConstants.get().ledCount / 2; i++) {
      ledStrip.setRGB(i, color.red, color.green, color.blue);
    }
  }

    private Timer purpleTimer = new Timer();

    public void setColorMovingDown(COLORS_467 fgColor, COLORS_467 bgColor, double speed) {
      if (purpleTimer.hasElapsed(
          speed * (RobotConstants.get().ledCount + 2))) {
        purpleTimer.reset();
      }

      for (int i = 0; i < RobotConstants.get().ledCount; i++) {
        if (purpleTimer.hasElapsed(speed * i)) {
          double timeUntilOff = Math.max(0, (speed * (i + 2)) - purpleTimer.get());
          double brightness = (255 * timeUntilOff);

          if (brightness == 0) {
            ledStrip.setLED(i, bgColor.getColor());

          } else {
            ledStrip.setRGB(
                i,
                (int) (fgColor.red * brightness),
                (int) (fgColor.green * brightness),
                (int) (fgColor.blue * brightness));
          }
        } else {
          ledStrip.setLED(i, bgColor.getColor());
        }
      }
    }

    public void setColorMovingUp(COLORS_467 fgColor, COLORS_467 bgColor, double speed) {
      if (purpleTimer.hasElapsed(
          speed * (RobotConstants.get().ledCount + 2))) {
        purpleTimer.reset();
      }

      for (int i = 0; i < RobotConstants.get().ledCount; i++) {
        int j = RobotConstants.get().ledCount - i - 1;
        if (purpleTimer.hasElapsed(speed * i)) {
          double timeUntilOff = Math.max(0, (speed * (i + 2)) - purpleTimer.get());
          double brightness = (255 * timeUntilOff);

          if (brightness == 0) {
            ledStrip.setLED(j, bgColor.getColor());

          } else {
            ledStrip.setRGB(
                j,
                (int) (fgColor.red * brightness),
                (int) (fgColor.green * brightness),
                (int) (fgColor.blue * brightness));
          }
        } else {
          ledStrip.setLED(j, bgColor.getColor());
        }
      }
    }

    public void setColorMovingUpTwoClr(Color topColor, Color bottomColor, double speed) {
      if (purpleTimer.hasElapsed(
          speed * (RobotConstants.get().ledCount + 2))) {
        purpleTimer.reset();
      }

      for (int i = RobotConstants.get().ledCount - 1; i >= 0; i--) {
        int j = RobotConstants.get().ledCount - 1 - i;
        if (purpleTimer.hasElapsed(speed * i)) {
          double timeUntilOff = Math.max(0, (speed * (i + 2)) - purpleTimer.get());
          double brightness = (255 * timeUntilOff);
          Color currentColor =
              j >= RobotConstants.get().ledCount / 2 ? topColor : bottomColor;

          if (brightness == 0) {
            ledStrip.setLED(j, currentColor);

          } else {
            ledStrip.setRGB(
                j,
                (int) (currentColor.red * brightness),
                (int) (currentColor.green * brightness),
                (int) (currentColor.blue * brightness));
          }
        } else {
          Color currentColor =
              j >= RobotConstants.get().ledCount / 2 ? topColor : bottomColor;
          ledStrip.setLED(j, currentColor);
        }
      }
    }

    public void setBlinkColors(COLORS_467 topColor, COLORS_467 bottomColor, Color bgColor) {

      if (purpleTimer.hasElapsed(0.6)) {
        purpleTimer.reset();
      } else if (purpleTimer.hasElapsed(0.25)) {
        setTop(topColor);
        setBottom(bottomColor);

      } else {
        set(bgColor);
      }
    }

    public void setAlternateColorsDown(COLORS_467 colorOne, COLORS_467 colorTwo, Color bgColor, double speed) {
      for (int i = 0; i < RobotConstants.get().ledCount; i++) {
        if (i % 2 == 0) {
          ledStrip.setLED(i, colorOne.getColor());
        } else {
          ledStrip.setLED(i, colorTwo.getColor());
        }
      }

      if (purpleTimer.hasElapsed(
          speed * (RobotConstants.get().ledCount + 2))) {
        purpleTimer.reset();
        for (int j = 0; j < RobotConstants.get().ledCount; j++) {
          ledStrip.setLED(j, bgColor);
        }
      }
    }

    public void setAlternateColorsUp(COLORS_467 colorOne, COLORS_467 colorTwo, Color bgColor) {
      for (int i = 0; i < RobotConstants.get().ledCount; i++) {
        if (i % 2 == 0) {
          ledStrip.setLED(i, colorOne.getColor());
        } else {
          ledStrip.setLED(i, colorTwo.getColor());
        }
      }

      for (int j = 0; j < RobotConstants.get().ledCount; j++) {
        int l = RobotConstants.get().ledCount - 1 - j;
        ledStrip.setLED(l, bgColor);
      }
    }

    public void setColorMovingDownTwoClr(Color topColor, Color bottomColor, double speed) {
      if (purpleTimer.hasElapsed(
          speed * (RobotConstants.get().ledCount + 2))) {
        purpleTimer.reset();
      }

      for (int i = 0; i < RobotConstants.get().ledCount; i++) {
        if (purpleTimer.hasElapsed(speed * i)) {
          double timeUntilOff = Math.max(0, (speed * (i + 2)) - purpleTimer.get());
          double brightness = (255 * timeUntilOff);

          if (brightness == 0) {
            if (i < RobotConstants.get().ledCount / 2) {
              ledStrip.setLED(i, topColor);
            } else {
              ledStrip.setLED(i, bottomColor);
            }

          } else {
            if (i < RobotConstants.get().ledCount / 2) {
              ledStrip.setRGB(
                  i,
                  (int) (topColor.red * brightness),
                  (int) (topColor.green * brightness),
                  (int) (topColor.blue * brightness));
            } else {
              ledStrip.setRGB(
                  i,
                  (int) (bottomColor.red * brightness),
                  (int) (bottomColor.green * brightness),
                  (int) (bottomColor.blue * brightness));
            }
          }
        } else {
          if (i < RobotConstants.get().ledCount / 2) {
            ledStrip.setLED(i, topColor);
          } else {
            ledStrip.setLED(i, bottomColor);
          }
        }
      }
    }
  

  private class Rainbows {

    private final double RAINBOW_TIMER_SPEED = 0.04;
    private final int RAINBOW_AMOUNT = 10;
    private double rainbowColor = 0;
    private Timer rainbowTimer = new Timer();

    public void setRainbowMovingUp() {
      if (rainbowTimer.hasElapsed(RAINBOW_TIMER_SPEED)) {
        rainbowColor -= RAINBOW_AMOUNT;

        if (rainbowColor > 360) rainbowColor = 0;
        rainbowTimer.reset();
      }

      for (int i = 0; i < RobotConstants.get().ledCount; i++) {
        ledStrip.setHSB(
            i,
            ((int) rainbowColor + (i * 360 / RobotConstants.get().ledCount)) % 360,
            255,
            127);
      }
    }

    public void setRainbowMovingDown() {
      if (rainbowTimer.hasElapsed(RAINBOW_TIMER_SPEED)) {
        rainbowColor += RAINBOW_AMOUNT;

        if (rainbowColor < 0) rainbowColor = 360;
        rainbowTimer.reset();
      }

      for (int i = 0; i < RobotConstants.get().ledCount; i++) {
        ledStrip.setHSB(
            i,
            ((int) rainbowColor + (i * 360 / RobotConstants.get().ledCount)) % 360,
            255,
            127);
      }
    }

    public void setRainbowMovingDownSecondInv() {
      if (rainbowTimer.hasElapsed(RAINBOW_TIMER_SPEED)) {
        rainbowColor += RAINBOW_AMOUNT;

        if (rainbowColor < 0) rainbowColor = 360;
        rainbowTimer.reset();
      }

      for (int i = 0; i < RobotConstants.get().ledCount; i++) {
        ledStrip.setLeftHSB(
            i,
            ((int) rainbowColor + (i * 360 / RobotConstants.get().ledCount)) % 360,
            255,
            127);
        ledStrip.setRightHSB(
            i,
            ((int) rainbowColor - (i * 360 / RobotConstants.get().ledCount)) % 360,
            255,
            127);
      }
    }

    public void setRainbow() {
      rainbowTimer.reset();
      for (int i = 0; i < RobotConstants.get().ledCount; i++) {
        ledStrip.setHSB(
            i,
            ((int) rainbowColor + (i * 360 / RobotConstants.get().ledCount)) % 360,
            255,
            127);
      }
    }
  }

  public class SetThirdLeds {
    private static final int topStart = 0;
    private static final int topEndandMidStart = (int) (RobotConstants.get().ledCount / 3);
    private static final int midEndandBottomStart =
        (int)
            (RobotConstants.get().ledCount - (RobotConstants.get().ledCount / 3));
    private static final int bottomEnd = (RobotConstants.get().ledCount);

    public void set(COLORS_467 color, int preSet) {
      // preSet = 1, 2, or 3. sets top 1/3, mid 1/3, or lower 1/3
      int start;
      int end;

      if (preSet == 1) {
        start = topStart;
        end = topEndandMidStart;
      } else if (preSet == 2) {
        start = topEndandMidStart;
        end = midEndandBottomStart - 1;

      } else {
        start = midEndandBottomStart;
        end = bottomEnd;
      }
      for (int i = start; i < end; i++) {
        ledStrip.setLED(i, color.getColor());
      }
    }
  }

  private class VictoryLeds {
    private COLORS_467 topClr;
    private COLORS_467 bottomClr;
    private boolean bright = false;
    private int brightness = 5;
    private static final int FADE_DURATION = 30;
    private static int fadeToWhite = 0;

    COLORS_467 fgColor;
    COLORS_467 bgColor;

    VictoryLeds(COLORS_467 fgColor, COLORS_467 bgColor) {
      this.fgColor = fgColor;
      this.bgColor = bgColor;
    }

    public void periodic() {
      if (topClr == null) {
        topClr = fgColor;
      }
      if (bottomClr == null) {
        bottomClr = bgColor;
      }
      if (brightness >= FADE_DURATION * 1.3 || bright) {
        if (brightness >= FADE_DURATION * 1.3) {
          if (topClr == fgColor) {
            topClr = bgColor;
            bottomClr = fgColor;
          } else {
            topClr = fgColor;
            bottomClr = bgColor;
          }
        }
        brightness = brightness - 2;
        bright = true;
      }
      if (brightness <= 5 || !bright) {
        brightness = brightness + 2;
        bright = false;
      }
      if (brightness > FADE_DURATION) {
        fadeToWhite = (brightness - FADE_DURATION) * 5;
      }
      for (int i = 0; i < RobotConstants.get().ledCount / 2; i++) {

        ledStrip.setRGB(
            i,
            Math.min((int) (topClr.red * brightness / FADE_DURATION) + fadeToWhite, 255),
            Math.min((int) (topClr.green * brightness / FADE_DURATION) + fadeToWhite, 255),
            Math.min((int) (topClr.blue * brightness / FADE_DURATION) + fadeToWhite, 255));
      }
      for (int i = (int) RobotConstants.get().ledCount / 2;
          i < RobotConstants.get().ledCount;
          i++) {

        ledStrip.setRGB(
            i,
            Math.min((int) (bottomClr.red * brightness / FADE_DURATION) + fadeToWhite, 255),
            Math.min((int) (bottomClr.green * brightness / FADE_DURATION) + fadeToWhite, 255),
            Math.min((int) (bottomClr.blue * brightness / FADE_DURATION) + fadeToWhite, 255));
      }
    }
  }
}