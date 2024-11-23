package frc.robot.dashboard;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

import java.util.function.Consumer;

public class TuningDashboardPIDController implements LoggedDashboardInput {
    private final LoggableInputs inputs = new LoggableInputs() {
        public void toLog(LogTable table) {
            table.put(key + "/kP", value.getP());
            table.put(key + "/kI", value.getI());
            table.put(key + "/kD", value.getD());
            table.put(key + "/iZone", value.getIZone());
        }

        public void fromLog(LogTable table) {
            var kP = table.get(key + "/kP", value.getP());
            var kI = table.get(key + "/kI", value.getI());
            var kD = table.get(key + "/kD", value.getD());
            var iZone = table.get(key + "/iZone", value.getIZone());
            if (kP != value.getP() || kI != value.getI() || kD != value.getD() || iZone != value.getIZone()) {
                value = new PIDController(kP, kI, kD);
                value.setIZone(iZone);
                configurator.accept(value);
            }
        }
    };

    private final String key;
    private final PIDConstants defaultValue;
    private final Consumer<PIDController> configurator;
    private PIDController value;
    private boolean currentlyShown = false;

    public TuningDashboardPIDController(DashboardSubsystem subsystem, String key, PIDConstants defaultValue) {
        this(subsystem, key, defaultValue, (pid) -> {
        });
    }

    public TuningDashboardPIDController(DashboardSubsystem subsystem, String key, PIDConstants defaultValue, Consumer<PIDController> configurator) {
        this.key = subsystem.prefix() + "/" + key;
        this.defaultValue = defaultValue;
        value = new PIDController(defaultValue.kP, defaultValue.kI, defaultValue.kD);
        value.setIZone(defaultValue.iZone);
        configurator.accept(value);
        this.configurator = configurator;

        periodic();
        Logger.registerDashboardInput(this);
    }

    public PIDController get() {
        return value;
    }

    public void periodic() {
        if (RobotState.tuningMode.get() && !currentlyShown) {
            SmartDashboard.putNumber(key + "/kP", SmartDashboard.getNumber(key + "/kP", value.getP()));
            SmartDashboard.putNumber(key + "/kI", SmartDashboard.getNumber(key + "/kI", value.getI()));
            SmartDashboard.putNumber(key + "/kD", SmartDashboard.getNumber(key + "/kD", value.getD()));
            SmartDashboard.putNumber(key + "/iZone", SmartDashboard.getNumber(key + "/iZone", value.getIZone()));
            currentlyShown = true;
        } else if (!RobotState.tuningMode.get() && currentlyShown) {
            // Only hide if it was changed
            if (value.getP() == defaultValue.kP && value.getI() == defaultValue.kI && value.getD() == defaultValue.kD && value.getIZone() == defaultValue.iZone) {
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
                var kP = SmartDashboard.getNumber(key + "/kP", value.getP());
                var kI = SmartDashboard.getNumber(key + "/kI", value.getI());
                var kD = SmartDashboard.getNumber(key + "/kD", value.getD());
                var iZone = SmartDashboard.getNumber(key + "/iZone", value.getIZone());
                if (kP != value.getP() || kI != value.getI() || kD != value.getD() || iZone != value.getIZone()) {
                    value = new PIDController(kP, kI, kD);
                    value.setIZone(iZone);
                    configurator.accept(value);
                }
            }
            Logger.processInputs(prefix, inputs);
        }
    }
}
