package frc.robot.subsystems.rollers;

import frc.robot.subsystems.drive.ModuleIO;
import org.littletonrobotics.junction.AutoLog;

public class RollersIO {
    @AutoLog
    public static class RollersIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    /**
     * Updates the set of loggable inputs.
     */
    public void updateInputs(ModuleIO.ModuleIOInputs inputs) {
    }

    /**
     * Enable or disable brake mode on the drive motor.
     */
    public void setBrakeMode(boolean enable) {
    }

    /**
     * Run the drive motor at the specified open loop value.
     */
    public void setOpenLoop(double output) {
    }

    /**
     * Run the turn motor to the specified rotation.
     */
    public void setVelocity(double velocityRadPerSec) {
    }
}
