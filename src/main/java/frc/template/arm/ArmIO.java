package frc.template.arm;

import com.pathplanner.lib.util.PIDConstants;
import org.littletonrobotics.junction.AutoLog;

public class ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public void updateInputs(ArmIOInputs inputs) {
    }

    public void setVoltage(double volts) {
    }

    public void setSetpoint(double setpointPositionRad, double ffVolts) {
    }

    public void setPosition(double currentPositionRad) {
    }

    public void setBrakeMode(boolean enabled) {
    }

    public void configurePID(PIDConstants pidConstants) {
    }

    public void setGearRatio(double gearRatio) {
    }
}
