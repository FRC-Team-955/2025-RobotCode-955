package frc.robot.dashboard;

import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public class TuningDashboardAnglularVelocity extends TuningDashboardNumber {
    public TuningDashboardAnglularVelocity(DashboardSubsystem subsystem, String key, AngularVelocity defaultValue) {
        super(subsystem, key + " (rad per sec)", defaultValue.in(RadiansPerSecond));
    }

    public AngularVelocity get() {
        return RadiansPerSecond.of(getRaw());
    }
}
