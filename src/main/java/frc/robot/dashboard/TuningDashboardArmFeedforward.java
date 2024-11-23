package frc.robot.dashboard;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

public class TuningDashboardArmFeedforward implements LoggedDashboardInput {
    private final LoggableInputs inputs = new LoggableInputs() {
        public void toLog(LogTable table) {
            table.put(key + "/ks", value.ks);
            table.put(key + "/kg", value.kg);
            table.put(key + "/kv", value.kv);
            table.put(key + "/ka", value.ka);
        }

        public void fromLog(LogTable table) {
            var ks = table.get(key + "/ks", value.ks);
            var kg = table.get(key + "/kg", value.kg);
            var kv = table.get(key + "/kv", value.kv);
            var ka = table.get(key + "/ka", value.ka);
            if (ks != value.ks || kg != value.kg || kv != value.kv || ka != value.ka) {
                value = new ArmFeedforward(ks, kg, kv, ka);
            }
        }
    };

    private final String key;
    private final ArmFeedforward defaultValue;
    private ArmFeedforward value;
    private boolean currentlyShown = false;

    public TuningDashboardArmFeedforward(DashboardSubsystem subsystem, String key, ArmFeedforward defaultValue) {
        this.key = subsystem.prefix() + "/" + key;
        this.defaultValue = defaultValue;
        this.value = defaultValue;

        periodic();
        Logger.registerDashboardInput(this);
    }

    public ArmFeedforward get() {
        return value;
    }

    public void periodic() {
        if (RobotState.tuningMode.get() && !currentlyShown) {
            SmartDashboard.putNumber(key + "/ks", SmartDashboard.getNumber(key + "/ks", value.ks));
            SmartDashboard.putNumber(key + "/kg", SmartDashboard.getNumber(key + "/kg", value.kg));
            SmartDashboard.putNumber(key + "/kv", SmartDashboard.getNumber(key + "/kv", value.kv));
            SmartDashboard.putNumber(key + "/ka", SmartDashboard.getNumber(key + "/ka", value.ka));
            currentlyShown = true;
        } else if (!RobotState.tuningMode.get() && currentlyShown) {
            // Only hide if it was changed
            if (value == defaultValue) {
                // unpublishing does nothing
                //SmartDashboard.getEntry(key + "/ks").unpublish();
                //SmartDashboard.getEntry(key + "/kg").unpublish();
                //SmartDashboard.getEntry(key + "/kv").unpublish();
                //SmartDashboard.getEntry(key + "/ka").unpublish();
                currentlyShown = false;
            }
        }

        if (currentlyShown) {
            if (!Logger.hasReplaySource()) {
                var ks = SmartDashboard.getNumber(key + "/ks", value.ks);
                var kg = SmartDashboard.getNumber(key + "/kg", value.kg);
                var kv = SmartDashboard.getNumber(key + "/kv", value.kv);
                var ka = SmartDashboard.getNumber(key + "/ka", value.ka);
                if (ks != value.ks || kg != value.kg || kv != value.kv || ka != value.ka) {
                    value = new ArmFeedforward(ks, kg, kv, ka);
                }
            }
            Logger.processInputs(prefix, inputs);
        }
    }
}
