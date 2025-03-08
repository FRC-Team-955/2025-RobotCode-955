package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.*;

public class LEDPatterns {
    private static final Color pink749 = new Color(255, 0, 128);

    private static final Color startupColor1 = Color.kRed;
    private static final Color startupColor2 = Color.kBlack;
    private static final double startupBreatheDuration = 1.0;

    /**
     * LEDPattern.breathe uses RobotController.getTime(), which doesn't really
     * work when the robot hasn't started up yet. Therefore, we must implement
     * the breathe algorithm ourselves. This is based off of 749's 2024 code.
     */
    public static void startup(AddressableLEDBuffer buffer) {
        double timestamp = System.currentTimeMillis() / 1000.0;
        double percent = timestamp % startupBreatheDuration / (startupBreatheDuration / 2.0);
        if (percent > 1) {
            percent = 1 - (percent - 1);
        }

        double interp = (Math.tanh((percent - 0.5) * 3.0) / Math.tanh(0.5 * 3.0)) * 0.5 + 0.5;

        double r = startupColor2.red + (startupColor1.red - startupColor2.red) * interp;
        double g = startupColor2.green + (startupColor1.green - startupColor2.green) * interp;
        double b = startupColor2.blue + (startupColor1.blue - startupColor2.blue) * interp;

        Color color = new Color(r, g, b);
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
    }

    public static final LEDPattern lowBatteryAlert = LEDPattern.solid(Color.kRed).blink(Seconds.of(0.5));
    public static final LEDPattern overrideAlert = LEDPattern.solid(Color.kBlue).blink(Seconds.of(0.5));

    public static final LEDPattern disabled = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, pink749)
            .scrollAtRelativeSpeed(Percent.per(Second).of(50));
    public static final LEDPattern disabledUnknown = LEDPattern.solid(Color.kPurple);

    public static final LEDPattern autoNotChosen = LEDPattern.solid(Color.kOrange).blink(Seconds.of(1));

    private static LEDPattern auto(Color allianceColor) {
        return LEDPattern.gradient(LEDPattern.GradientType.kContinuous, allianceColor, Color.kWhite, Color.kWhite, Color.kWhite)
                .scrollAtRelativeSpeed(Percent.per(Second).of(250));
    }

    public static final LEDPattern autoRed = auto(Color.kRed);
    public static final LEDPattern autoBlue = auto(Color.kBlue);
    public static final LEDPattern autoUnknown = auto(Color.kPurple);
    public static final LEDPattern autoFinished = LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.5));

    public static final LEDPattern eject = LEDPattern.solid(Color.kRed).blink(Seconds.of(0.1));
    public static final LEDPattern finalizing = LEDPattern.solid(Color.kGreen);
    public static final LEDPattern driverConfirm = LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.1));
    public static final LEDPattern autoScoring = LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Percent.per(Second).of(150));
    public static final LEDPattern waitElevator = LEDPattern.solid(Color.kOrange).blink(Seconds.of(0.25));
    public static final LEDPattern funnelIntaking = LEDPattern.solid(Color.kOrange).blink(Seconds.of(0.1));
    public static final LEDPattern endgameAlert = LEDPattern.solid(Color.kAqua).blink(Seconds.of(0.1));
    public static final LEDPattern funnelTriggered = LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.1));
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