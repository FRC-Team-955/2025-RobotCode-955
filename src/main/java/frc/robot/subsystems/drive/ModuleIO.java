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

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public class ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public boolean turnEncoderConnected = false;
        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public double[] odometryTimestamps = new double[]{};
        public double[] odometryDrivePositionsRad = new double[]{};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[]{};
    }

    /**
     * Updates the set of loggable inputs.
     */
    public void updateInputs(ModuleIOInputs inputs) {
    }

    /**
     * Run the drive motor at the specified open loop value.
     */
    public void setDriveVoltage(double volts) {
    }

    /**
     * Run the turn motor at the specified open loop value.
     */
    public void setTurnVoltage(double volts) {
    }

    /**
     * Run the drive motor at the specified velocity.
     */
    public void setDriveVelocity(double velocityRadPerSec) {
    }

    /**
     * Run the turn motor to the specified rotation.
     */
    public void setTurnPosition(Rotation2d rotation) {
    }
}
