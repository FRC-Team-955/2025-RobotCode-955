package frc.robot.subsystems.intakeRoller;

import frc.lib.PIDF;
import org.littletonrobotics.junction.AutoLog;

public class IntakeRollerIO {
    @AutoLog
    public static class IntakeRollerIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double temperatureCelsius = 0.0;
        public boolean isCoralIn = false;
    }

    public void updateInputs(IntakeRollerIOInputs inputs) {
    }

    public void setPositionPIDF(PIDF newGains) {
    }

    public void setVelocityPIDF(PIDF newGains) {
    }

    public void setBrakeMode(boolean enable) {
    }

    public void setOpenLoop(double output) {
    }

    public void setVelocity(double velocityRadPerSec) {
    }


    public void spawnCoral() {

    }

    public void setPosition(double positionRad) {
    }
}

