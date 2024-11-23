package frc.robot.dashboard;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

import java.util.function.Consumer;

public class TuningDashboardPIDConstants implements LoggedDashboardInput {
    private final LoggableInputs inputs = new LoggableInputs() {
        public void toLog(LogTable table) {
            table.put(key + "/kP", value.kP);
            table.put(key + "/kI", value.kI);
            table.put(key + "/kD", value.kD);
            table.put(key + "/iZone", value.iZone);
        }

        public void fromLog(LogTable table) {
            var kP = table.get(key + "/kP", value.kP);
            var kI = table.get(key + "/kI", value.kI);
            var kD = table.get(key + "/kD", value.kD);
            var iZone = table.get(key + "/iZone", value.iZone);
            if (kP != value.kP || kI != value.kI || kD != value.kD || iZone != value.iZone) {
                value = new PIDConstants(kP, kI, kD, iZone);
                previouslyChanged = true;
            }
        }
    };

    private final String key;
    private final PIDConstants defaultValue;
    private PIDConstants value;
    private boolean previouslyChanged = false;
    private boolean currentlyShown = false;

    public TuningDashboardPIDConstants(DashboardSubsystem subsystem, String key, PIDConstants defaultValue) {
        this.key = subsystem.prefix() + "/" + key;
        this.defaultValue = defaultValue;
        this.value = defaultValue;

        periodic();
        Logger.registerDashboardInput(this);
    }

    public PIDConstants get() {
        return value;
    }

    public void ifChanged(Consumer<PIDConstants> ifChanged) {
        if (previouslyChanged) {
            previouslyChanged = false;
            ifChanged.accept(value);
        }
    }

    public void periodic() {
        if (RobotState.tuningMode.get() && !currentlyShown) {
            SmartDashboard.putNumber(key + "/kP", SmartDashboard.getNumber(key + "/kP", value.kP));
            SmartDashboard.putNumber(key + "/kI", SmartDashboard.getNumber(key + "/kI", value.kI));
            SmartDashboard.putNumber(key + "/kD", SmartDashboard.getNumber(key + "/kD", value.kD));
            SmartDashboard.putNumber(key + "/iZone", SmartDashboard.getNumber(key + "/iZone", value.iZone));
            currentlyShown = true;
        } else if (!RobotState.tuningMode.get() && currentlyShown) {
            // Only hide if it was changed
            if (value == defaultValue) {
                // unpublishing does nothing
                //SmartDashboard.getEntry(key + "/kP").unpublish();
                //SmartDashboard.getEntry(key + "/kI").unpublish();
                //SmartDashboard.getEntry(key + "/kD").unpublish();
                //SmartDashboard.getEntry(key + "/iZone").unpublish();
                currentlyShown = false;
            }
        }

        if (currentlyShown) {
            if (!Logger.hasReplaySource()) {
                var kP = SmartDashboard.getNumber(key + "/kP", value.kP);
                var kI = SmartDashboard.getNumber(key + "/kI", value.kI);
                var kD = SmartDashboard.getNumber(key + "/kD", value.kD);
                var iZone = SmartDashboard.getNumber(key + "/iZone", value.iZone);
                if (kP != value.kP || kI != value.kI || kD != value.kD || iZone != value.iZone) {
                    value = new PIDConstants(kP, kI, kD, iZone);
                    previouslyChanged = true;
                }
            }
            Logger.processInputs(prefix, inputs);
        }
    }
}
