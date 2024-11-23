package frc.template.wheel;

import com.pathplanner.lib.util.PIDConstants;
import org.littletonrobotics.junction.AutoLog;

public class WheelIO {
    @AutoLog
    public static class WheelIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public void updateInputs(WheelIOInputs inputs) {
    }

    public void setVoltage(double volts) {
    }

    public void setSetpoint(double setpointVelocityRadPerSec, double ffVolts) {
    }

    public void setBrakeMode(boolean enabled) {
    }

    public void configurePID(PIDConstants pidConstants) {
    }

    public void setGearRatio(double gearRatio) {
    }
}
