package frc.robot.subsystems.intakerollers;

import frc.lib.PIDF;
import org.littletonrobotics.junction.AutoLog;

public class IntakeRollersIO {
    @AutoLog
    public static class IntakeRollersIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double temperatureCelsius = 0.0;
        public boolean isNoteInsideIntake = false;
    }

    public void updateInputs(IntakeRollersIOInputs inputs) {}

//    public void setPositionPIDF(PIDF newGains) {}

    public void setVelocityPIDF(PIDF newGains) {}

    public void setBrakeMode(boolean enable) {}

    public void setOpenLoop(double output) {}

    public void setVelocity(double velocityRadPerSec) {}

    public void setPosition(double positionRad) {}

    public void setRunning(boolean runIntake) {}

    public void launchNote() {}
}
