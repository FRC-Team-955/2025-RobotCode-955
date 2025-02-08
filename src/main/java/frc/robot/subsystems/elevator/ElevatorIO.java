package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public class ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;

        public boolean limitSwitchConnected = false;
        public boolean limitSwitchTriggered = false;
    }

    public void updateInputs(ElevatorIOInputs inputs) {
    }

    /**
     * Enable or disable brake mode on the drive motor.
     */
    public void setBrakeMode(boolean enable) {
    }

    /**
     * Run the motor at the specified open loop value.
     */
    public void setOpenLoop(double output) {
    }

    /**
     * Run the motor to the specified position.
     * Should use feedback to go to the specified position and use feedforward to go to the specified velocity
     */
    public void setClosedLoop(double positionRad, double velocityRadPerSec) {
    }

    /** Set the encoder of the elevator to the specified position. Used for zeroing */
    public void setEncoder(double positionRad) {
    }
}