package frc.robot.subsystems.intakepivot;

import frc.lib.PIDF;
import org.littletonrobotics.junction.AutoLog;

public class IntakePivotIO {
    @AutoLog
    public static class IntakePivotIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double temperatureCelsius;

        public boolean absoluteEncoderConnected = false;
        public double absolutePositionRad = 0.0;
    }

    public void updateInputs(IntakePivotIOInputs inputs) {}

    public void setBrakeMode(boolean enable) {}

    public void setOpenLoop(double output) {}

    public void setClosedLoop(double positionRad) {}

    public void setIntakePIDF(PIDF newGains) {}
}
