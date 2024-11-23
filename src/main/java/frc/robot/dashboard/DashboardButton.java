package frc.robot.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

public class DashboardButton implements LoggedDashboardInput {
    private final LoggableInputs inputs = new LoggableInputs() {
        public void toLog(LogTable table) {
            table.put(key, pressed);
        }

        public void fromLog(LogTable table) {
            pressed = table.get(key, pressed);
        }
    };

    private final String key;
    private boolean pressed = false;

    public DashboardButton(DashboardSubsystem subsystem, String key) {
        this.key = subsystem.prefix() + "/" + key;

        SmartDashboard.putBoolean(this.key, SmartDashboard.getBoolean(this.key, pressed));
        periodic();
        Logger.registerDashboardInput(this);
    }

    public Trigger trigger() {
        return new Trigger(() -> pressed);
    }

    public void periodic() {
        if (pressed) {
            // Do this before updating pressed so that one loop cycle goes by before we say it's not pressed
            // This allows any registered triggers to trigger
            pressed = false;
            SmartDashboard.putBoolean(key, pressed);
        } else if (!Logger.hasReplaySource()) {
            // No point in checking if we just undid the press
            pressed = SmartDashboard.getBoolean(key, pressed);
        }
        Logger.processInputs(prefix, inputs);
    }
}
