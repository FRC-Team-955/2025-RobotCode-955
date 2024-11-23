package frc.robot.dashboard;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public class TuningDashboardAnglularVelocity extends TuningDashboardNumber {
    public TuningDashboardAnglularVelocity(DashboardSubsystem subsystem, String key, Measure<Velocity<Angle>> defaultValue) {
        super(subsystem, key + " (rad per sec)", defaultValue.in(RadiansPerSecond));
    }

    public Measure<Velocity<Angle>> get() {
        return RadiansPerSecond.of(getRaw());
    }
}
