package frc.robot.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

/**
 * Dashboard number that only appears in tuning mode.
 */
public class TuningDashboardNumber implements LoggedDashboardInput {
    private final LoggableInputs inputs = new LoggableInputs() {
        public void toLog(LogTable table) {
            table.put(key, value);
        }

        public void fromLog(LogTable table) {
            value = table.get(key, value);
        }
    };

    private final String key;
    private final double defaultValue;
    private double value;
    private boolean currentlyShown = false;

    public TuningDashboardNumber(DashboardSubsystem subsystem, String key, double defaultValue) {
        this.key = subsystem.prefix() + "/" + key;
        this.defaultValue = defaultValue;
        this.value = defaultValue;

        periodic();
        Logger.registerDashboardInput(this);
    }

    public double getRaw() {
        return value;
    }

    public void periodic() {
        if (RobotState.tuningMode.get() && !currentlyShown) {
            SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, value));
            currentlyShown = true;
        } else if (!RobotState.tuningMode.get() && currentlyShown) {
            // Only hide if it was changed
            if (value == defaultValue) {
                // unpublishing does nothing
                //SmartDashboard.getEntry(key).unpublish();
                currentlyShown = false;
            }
        }

        if (currentlyShown) {
            if (!Logger.hasReplaySource()) {
                value = SmartDashboard.getNumber(key, value);
            }
            Logger.processInputs(prefix, inputs);
        }
    }
}
