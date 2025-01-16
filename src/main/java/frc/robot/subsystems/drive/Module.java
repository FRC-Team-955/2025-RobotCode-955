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
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private final Alert turnEncoderDisconnectedAlert;
    /**
     * -- GETTER --
     * Returns the module positions received this cycle.
     */
    @Getter
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[]{};

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;

        driveDisconnectedAlert = new Alert("Disconnected drive motor on module " + index + ".", AlertType.kError);
        turnDisconnectedAlert = new Alert("Disconnected turn motor on module " + index + ".", AlertType.kError);
        turnEncoderDisconnectedAlert = new Alert("Disconnected turn encoder on module " + index + ".", AlertType.kError);
    }

    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + index, inputs);

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
        turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
    }

    public void periodicAfterCommands() {
        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i] * driveConfig.wheelRadiusMeters();
            double angle = inputs.odometryTurnPositionsRad[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, new Rotation2d(angle));
        }
    }

    /**
     * Runs the module with the specified setpoint state. Mutates the state to optimize it.
     */
    public void runSetpoint(SwerveModuleState state) {
        // Optimize velocity setpoint
        var currentAngle = new Rotation2d(inputs.turnPositionRad);
        state.optimize(currentAngle);
        state.cosineScale(currentAngle);

        // Apply setpoints
        io.setDriveVelocity(state.speedMetersPerSecond / driveConfig.wheelRadiusMeters());
        io.setTurnPosition(state.angle.getRadians());
    }

    /**
     * Runs the module with the specified output while controlling to zero degrees.
     */
    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(0.0);
    }

    /**
     * Disables all outputs to motors.
     */
    public void stop() {
        io.setDriveOpenLoop(0.0);
        io.setTurnOpenLoop(0.0);
    }

    /**
     * Returns the current turn angle of the module.
     */
    public Rotation2d getAngle() {
        return new Rotation2d(inputs.turnPositionRad);
    }

    public double getPositionRad() {
        return inputs.drivePositionRad;
    }

    /**
     * Returns the current drive position of the module in meters.
     */
    public double getPositionMeters() {
        return inputs.drivePositionRad * driveConfig.wheelRadiusMeters();
    }

    /**
     * Returns the current drive velocity of the module in meters per second.
     */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * driveConfig.wheelRadiusMeters();
    }

    /**
     * Returns the module position (turn angle and drive position).
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /**
     * Returns the module state (turn angle and drive velocity).
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /**
     * Returns the timestamps of the samples received this cycle.
     */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    /**
     * Returns the module position in radians.
     */
    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drivePositionRad;
    }

    /**
     * Returns the module velocity in rotations/sec (Phoenix native units).
     */
    public double getFFCharacterizationVelocity() {
        return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
    }
}
