package frc.robot.dashboard;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class TuningDashboardVelocity extends TuningDashboardNumber {
    public TuningDashboardVelocity(DashboardSubsystem subsystem, String key, Measure<Velocity<Distance>> defaultValue) {
        super(subsystem, key + " (meters per second)", defaultValue.in(MetersPerSecond));
    }

    public Measure<Velocity<Distance>> get() {
        return MetersPerSecond.of(getRaw());
    }
}
