package frc.robot.dashboard;

import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;

public class DashboardAngle extends DashboardNumber {
    public DashboardAngle(DashboardSubsystem subsystem, String key, Angle defaultValue) {
        super(subsystem, key + " (degrees)", defaultValue.in(Degrees));
    }

    public Angle get() {
        return Degrees.of(getRaw());
    }
}
