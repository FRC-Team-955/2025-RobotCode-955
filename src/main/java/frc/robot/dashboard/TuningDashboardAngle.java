package frc.robot.dashboard;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Measure;

import static edu.wpi.first.units.Units.Degrees;

public class TuningDashboardAngle extends TuningDashboardNumber {
    public TuningDashboardAngle(DashboardSubsystem subsystem, String key, Angle defaultValue) {
        super(subsystem, key + " (degrees)", defaultValue.in(Degrees));
    }

    public Angle get() {
        return Degrees.of(getRaw());
    }
}
