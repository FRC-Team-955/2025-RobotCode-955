package frc.lib.motor;

import frc.lib.PIDF;
import org.littletonrobotics.junction.AutoLog;

public class MotorIO {
    public void updateInputs(MotorIOInputs inputs) {}

    public void setPositionPIDF(PIDF newGains) {}

    public void setVelocityPIDF(PIDF newGains) {}

    public void setBrakeMode(boolean enable) {}

    public void setRequest(RequestType type, double value) {}

    @AutoLog
    public static class MotorIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double temperatureCelsius = 0.0;
    }
}
