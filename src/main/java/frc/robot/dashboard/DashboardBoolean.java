package frc.robot.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

public class DashboardBoolean implements LoggedDashboardInput {
    private final LoggableInputs inputs = new LoggableInputs() {
        public void toLog(LogTable table) {
            table.put(key, value);
        }

        public void fromLog(LogTable table) {
            value = table.get(key, value);
        }
    };

    private final String key;
    private boolean value;

    public DashboardBoolean(DashboardSubsystem subsystem, String key, boolean defaultValue) {
        this.key = subsystem.prefix() + "/" + key;
        this.value = defaultValue;

        SmartDashboard.putBoolean(this.key, SmartDashboard.getBoolean(this.key, value));
        periodic();
        Logger.registerDashboardInput(this);
    }

    public boolean get() {
        return value;
    }

    public void periodic() {
        if (!Logger.hasReplaySource()) {
            value = SmartDashboard.getBoolean(key, value);
        }
        Logger.processInputs(prefix, inputs);
    }
}
