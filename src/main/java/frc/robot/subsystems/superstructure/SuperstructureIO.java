package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;

public class SuperstructureIO {
    @AutoLog
    public static class SuperstructureIOInputs {
        public boolean intakeRangeConnected = false;
        public double intakeRangeMeters = Double.MAX_VALUE;
    }

    public void updateInputs(SuperstructureIOInputs inputs) {}
}
