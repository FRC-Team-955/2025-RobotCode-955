package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

import java.util.function.Supplier;

public class DashboardColors {
    public static final Supplier<Color> disabled = () -> DriverStation.getAlliance()
            .map(alliance -> alliance == DriverStation.Alliance.Blue ? Color.kFirstBlue : Color.kFirstRed)
            .orElse(Color.kPurple);

    public static final Supplier<Color> eject = blink(Color.kRed, 0.5);
    public static final Supplier<Color> finalizing = blink(Color.kGreen, 0.5);
    public static final Supplier<Color> driverConfirm = blink(Color.kYellow, 0.5);
    public static final Supplier<Color> autoScoring = blink(Color.kCyan, 0.5);
    public static final Supplier<Color> waitElevator = blink(Color.kOrange, 0.5);
    public static final Supplier<Color> funnelIntaking = blink(Color.kOrange, 0.5);

    private static Supplier<Color> blink(Color color, double secondsForOn) {
        return () -> Timer.getTimestamp() % secondsForOn * 2.0 < secondsForOn
                ? color
                : Color.kBlack;
    }
}
