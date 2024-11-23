package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public class GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean isConnected = false;
        public double yawPositionRad = 0.0;
        public double yawVelocityRadPerSec = 0.0;
    }

    public void updateInputs(GyroIOInputs inputs) {
    }
}
