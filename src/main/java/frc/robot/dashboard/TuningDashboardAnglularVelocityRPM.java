package frc.robot.dashboard;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Measure;
import edu.wpi.first.units.measure.Velocity;

import static edu.wpi.first.units.Units.RPM;

public class TuningDashboardAnglularVelocityRPM extends TuningDashboardNumber {
    public TuningDashboardAnglularVelocityRPM(DashboardSubsystem subsystem, String key, AngularVelocity defaultValue) {
        super(subsystem, key + " (RPM)", defaultValue.in(RPM));
    }

    public AngularVelocity get() {
        return RPM.of(getRaw());
    }
}
