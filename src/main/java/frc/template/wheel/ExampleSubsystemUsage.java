package frc.template.wheel;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public final class ExampleSubsystemUsage {
    private static final double WHEEL_SETPOINT_TOLERANCE = Units.rotationsPerMinuteToRadiansPerSecond(10);

    private final WheelIOInputsAutoLogged wheelInputs = new WheelIOInputsAutoLogged();
    private final WheelIO wheelIO;

    private final SimpleMotorFeedforward wheelFeedforward;
    private Double wheelSetpointRadPerSec = null;

    /**
     * @param gearRatio >1 means a reduction, <1 means a upduction
     */
    private ExampleSubsystemUsage(
            WheelIO wheelIO,
            SimpleMotorFeedforward wheelFeedforward,
            PIDConstants pidConstants,
            double gearRatio
    ) {
        this.wheelIO = wheelIO;
        this.wheelFeedforward = wheelFeedforward;

        wheelIO.configurePID(pidConstants);
        wheelIO.setGearRatio(gearRatio);
    }

    public void periodic() {
        wheelIO.updateInputs(wheelInputs);
        Logger.processInputs("Inputs/ExampleSubsystem", wheelInputs);

        Logger.recordOutput("ExampleSubsystem/ClosedLoop", wheelSetpointRadPerSec != null);
        if (wheelSetpointRadPerSec != null) {
            Logger.recordOutput("ExampleSubsystem/Setpoint", wheelSetpointRadPerSec);

            if (DriverStation.isEnabled()) {
                var ffVolts = wheelFeedforward.calculate(wheelSetpointRadPerSec, 0);
                Logger.recordOutput("ExampleSubsystem/FFVolts", ffVolts);
                wheelIO.setSetpoint(wheelSetpointRadPerSec, ffVolts);
            }
        }
    }

    public void setPercent(double percent) {
        wheelIO.setVoltage(percent * 12);
        wheelSetpointRadPerSec = null;
    }

    /**
     * 0 means parallel to the ground
     */
    public void setSetpoint(Measure<Velocity<Angle>> setpoint) {
        wheelSetpointRadPerSec = setpoint.in(RadiansPerSecond);
    }

    public boolean atSetpoint() {
        return Math.abs(wheelInputs.velocityRadPerSec - wheelSetpointRadPerSec) <= WHEEL_SETPOINT_TOLERANCE;
    }

    public void stop() {
        wheelIO.setVoltage(0);
        wheelSetpointRadPerSec = null;
    }

    public void setBreakMode(boolean enabled) {
        wheelIO.setBrakeMode(enabled);
    }
}
