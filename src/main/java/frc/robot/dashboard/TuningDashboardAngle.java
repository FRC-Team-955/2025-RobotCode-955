package frc.robot.dashboard;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;

import static edu.wpi.first.units.Units.Degrees;

public class TuningDashboardAngle extends TuningDashboardNumber {
    public TuningDashboardAngle(DashboardSubsystem subsystem, String key, Measure<Angle> defaultValue) {
        super(subsystem, key + " (degrees)", defaultValue.in(Degrees));
    }

    public Measure<Angle> get() {
        return Degrees.of(getRaw());
    }
}
