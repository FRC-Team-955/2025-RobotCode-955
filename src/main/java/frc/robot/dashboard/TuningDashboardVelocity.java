package frc.robot.dashboard;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Measure;
import edu.wpi.first.units.measure.Velocity;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class TuningDashboardVelocity extends TuningDashboardNumber {
    public TuningDashboardVelocity(DashboardSubsystem subsystem, String key, LinearVelocity defaultValue) {
        super(subsystem, key + " (meters per second)", defaultValue.in(MetersPerSecond));
    }

    public LinearVelocity get() {
        return MetersPerSecond.of(getRaw());
    }
}
