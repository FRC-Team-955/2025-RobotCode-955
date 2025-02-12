// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public class ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public boolean turnAbsoluteEncoderConnected = false;
        public double turnAbsolutePositionRad = 0.0;

        public double[] odometryDriveTimestamps = new double[]{};
        public double[] odometryDrivePositionsRad = new double[]{};

        public double[] odometryTurnTimestamps = new double[]{};
        public double[] odometryTurnPositionsRad = new double[]{};
    }

    /**
     * Updates the set of loggable inputs.
     */
    public void updateInputs(ModuleIOInputs inputs) {
    }

    /**
     * Enable or disable brake mode on the drive motor.
     */
    public void setDriveBrakeMode(boolean enable) {
    }

    /**
     * Enable or disable brake mode on the turn motor.
     */
    public void setTurnBrakeMode(boolean enable) {
    }

    /**
     * Run the drive motor at the specified open loop value.
     */
    public void setDriveOpenLoop(double output) {
    }

    /**
     * Run the turn motor at the specified open loop value.
     */
    public void setTurnOpenLoop(double output) {
    }

    /**
     * Run the drive motor at the specified velocity.
     */
    public void setDriveVelocity(double velocityRadPerSec) {
    }

    /**
     * Run the turn motor to the specified rotation.
     */
    public void setTurnPosition(double positionRad) {
    }
}
