package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;

public class SuperstructureIO {
    @AutoLog
    public static class SuperstructureIOInputs {
        public boolean intakeRangeConnected = false;
        public double intakeRangeMeters = 0.0;

        public boolean indexerBeamBreakConnected = false;
        /** If triggered is true, the beam is broken */
        public boolean indexerBeamBreakTriggered = false;

        public boolean endEffectorBeamBreakConnected = false;
        /** If triggered is true, the beam is broken */
        public boolean endEffectorBeamBreakTriggered = false;
    }

    public void updateInputs(SuperstructureIOInputs inputs) {
    }
}