package frc.robot.dashboard;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

public class TuningDashboardSimpleFeedforward implements LoggedDashboardInput {
    private final LoggableInputs inputs = new LoggableInputs() {
        public void toLog(LogTable table) {
            table.put(key + "/ks", value.ks);
            table.put(key + "/kv", value.kv);
            table.put(key + "/ka", value.ka);
        }

        public void fromLog(LogTable table) {
            var ks = table.get(key + "/ks", value.ks);
            var kv = table.get(key + "/kv", value.kv);
            var ka = table.get(key + "/ka", value.ka);
            if (ks != value.ks || kv != value.kv || ka != value.ka) {
                value = new SimpleMotorFeedforward(ks, kv, ka);
            }
        }
    };

    private final String key;
    private final SimpleMotorFeedforward defaultValue;
    private SimpleMotorFeedforward value;
    private boolean currentlyShown = false;

    public TuningDashboardSimpleFeedforward(DashboardSubsystem subsystem, String key, SimpleMotorFeedforward defaultValue) {
        this.key = subsystem.prefix() + "/" + key;
        this.defaultValue = defaultValue;
        this.value = defaultValue;

        periodic();
        Logger.registerDashboardInput(this);
    }

    public SimpleMotorFeedforward get() {
        return value;
    }

    public void periodic() {
        if (RobotState.tuningMode.get() && !currentlyShown) {
            SmartDashboard.putNumber(key + "/ks", SmartDashboard.getNumber(key + "/ks", value.ks));
            SmartDashboard.putNumber(key + "/kv", SmartDashboard.getNumber(key + "/kv", value.kv));
            SmartDashboard.putNumber(key + "/ka", SmartDashboard.getNumber(key + "/ka", value.ka));
            currentlyShown = true;
        } else if (!RobotState.tuningMode.get() && currentlyShown) {
            // Only hide if it was changed
            if (value == defaultValue) {
                // unpublishing does nothing
                //SmartDashboard.getEntry(key + "/ks").unpublish();
                //SmartDashboard.getEntry(key + "/kv").unpublish();
                //SmartDashboard.getEntry(key + "/ka").unpublish();
                currentlyShown = false;
            }
        }

        if (currentlyShown) {
            if (!Logger.hasReplaySource()) {
                var ks = SmartDashboard.getNumber(key + "/ks", value.ks);
                var kv = SmartDashboard.getNumber(key + "/kv", value.kv);
                var ka = SmartDashboard.getNumber(key + "/ka", value.ka);
                if (ks != value.ks || kv != value.kv || ka != value.ka) {
                    value = new SimpleMotorFeedforward(ks, kv, ka);
                }
            }
            Logger.processInputs(prefix, inputs);
        }
    }
}
