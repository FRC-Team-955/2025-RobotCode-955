package frc.robot.subsystems.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveConstants;

import static edu.wpi.first.units.Units.*;

public class LEDPatterns {
    private static final Color pink749 = new Color(255, 0, 128);

    public static final boolean constantSet = Constants.tuningMode || DriveConstants.disableDriving || DriveConstants.disableGyro;

    /**
     * LEDPattern.breathe uses RobotController.getTime(), which doesn't really
     * work when the robot hasn't started up yet. Therefore, we must implement
     * the breathe algorithm ourselves. This is based off of 749's 2024 code.
     */
    public static LEDPattern startup() {
        Color color1 = constantSet ? Color.kGreen : Color.kRed;
        Color color2 = Color.kBlack;
        double period = 1.0;

        return (reader, writer) -> {
            double timestamp = System.currentTimeMillis() / 1000.0;
            double percent = timestamp % period / (period / 2.0);
            if (percent > 1) {
                percent = 1 - (percent - 1);
            }

            double interp = (Math.tanh((percent - 0.5) * 3.0) / Math.tanh(0.5 * 3.0)) * 0.5 + 0.5;

            double r = color2.red + (color1.red - color2.red) * interp;
            double g = color2.green + (color1.green - color2.green) * interp;
            double b = color2.blue + (color1.blue - color2.blue) * interp;

            Color color = new Color(r, g, b);
            for (int i = 0; i < reader.getLength(); i++) {
                writer.setLED(i, color);
            }
        };
    }

    /** Based on 749's 2024 code */
    private static LEDPattern wave(Color backgroundColor, Color waveColor, double periodSeconds) {
        double lengthMultiplier = 4.0;
        double sinMultiplier = 2.0 / lengthMultiplier;
        double periodMicros = periodSeconds * 1_000_000.0;

        return (reader, writer) -> {
            int length = reader.getLength();
            double extendedLength = lengthMultiplier * length;

            double percent = RobotController.getTime() % periodMicros / periodMicros;
            long offset = Math.round(percent * extendedLength);
            for (int i = 0; i < length; i++) {
                double localPercent = (i - offset) % (extendedLength) / length;

                double interp = Math.sin(sinMultiplier * Math.PI * localPercent);

                double r = MathUtil.interpolate(backgroundColor.red, waveColor.red, interp);
                double g = MathUtil.interpolate(backgroundColor.green, waveColor.green, interp);
                double b = MathUtil.interpolate(backgroundColor.blue, waveColor.blue, interp);
                writer.setLED(i, new Color(r, g, b));
            }
        };
    }

    public static final LEDPattern lowBatteryAlert = LEDPattern.solid(Color.kRed).blink(Seconds.of(0.5));
    public static final LEDPattern overrideAlert = LEDPattern.solid(Color.kBlue).blink(Seconds.of(0.5));
    public static final LEDPattern constantSetAlert = LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.5));

    public static final LEDPattern disabled = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kRed, Color.kRed, pink749)
            .scrollAtRelativeSpeed(Percent.per(Second).of(75));
    public static final LEDPattern disabledUnknown = LEDPattern.solid(Color.kPurple);

    public static final LEDPattern autoNotChosen = LEDPattern.solid(Color.kOrangeRed).blink(Seconds.of(1));

    private static LEDPattern auto(Color allianceColor) {
        return wave(Color.kWhite, allianceColor, 0.4);
    }

    public static final LEDPattern autoRed = auto(Color.kRed);
    public static final LEDPattern autoBlue = auto(Color.kBlue);
    public static final LEDPattern autoUnknown = auto(Color.kPurple);
    public static final LEDPattern autoFinished = LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.5));

    public static final LEDPattern eject = LEDPattern.solid(Color.kRed).blink(Seconds.of(0.1));
    public static final LEDPattern finalizing = LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.1));
    public static final LEDPattern driverConfirm = LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.1));
    public static final LEDPattern autoScoring = LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Percent.per(Second).of(150));
    public static final LEDPattern waitElevator = LEDPattern.solid(Color.kOrangeRed).blink(Seconds.of(0.25));
    public static final LEDPattern funnelIntaking = LEDPattern.solid(Color.kOrangeRed).blink(Seconds.of(0.1));
    public static final LEDPattern endgameAlert = LEDPattern.solid(Color.kAqua).blink(Seconds.of(0.1));
    public static final LEDPattern endEffectorTriggered = LEDPattern.solid(Color.kGreen);
    public static final LEDPattern idle = LEDPattern.kOff;

    public static LEDPattern patternForAlliance(LEDPattern redAndBlue, LEDPattern unknown) {
        return DriverStation.getAlliance().isPresent()
                ? redAndBlue
                : unknown;
    }

    public static LEDPattern patternForAlliance(LEDPattern red, LEDPattern blue, LEDPattern unknown) {
        return DriverStation.getAlliance()
                .map(alliance -> alliance == DriverStation.Alliance.Blue ? blue : red)
                .orElse(unknown);
    }
}