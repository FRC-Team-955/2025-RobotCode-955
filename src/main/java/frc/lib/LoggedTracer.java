package frc.lib;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class LoggedTracer {
    public static double startTime = -1.0;

    private LoggedTracer() {}

    public static void reset() {
        startTime = Timer.getFPGATimestamp();
    }

    public static void record(String epochName) {
        double now = Timer.getFPGATimestamp();
        Logger.recordOutput("LoggedTracer/" + epochName + "MS", (now - startTime) * 1000.0);
        startTime = now;
    }
}
