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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.drive.DriveConstants.moduleConfig;
import static frc.robot.util.SparkUtil.*;

/**
 * Module IO implementation for Spark Max drive motor controller, Spark Max turn motor controller,
 * and CANcoder.
 */
public class ModuleIOAlphabot extends ModuleIO {
    private static final Alert turnRelativeEncoderNotReset = new Alert("One or more alpha drive modules has not successfully reset their relative turn encoder", Alert.AlertType.kError);

    private final double zeroRotationRad;

    // Hardware objects
    private final SparkMax driveSpark;
    private final SparkMax turnSpark;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;
    private final SparkMaxConfig driveConfig;
    private final SparkMaxConfig turnConfig;
    private final CANcoder cancoder;

    // Closed loop controllers
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;

    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    private final StatusSignal<Angle> turnAbsolutePosition;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

    private final SimpleMotorFeedforward driveFF = moduleConfig.driveGains().toSimpleFF();

    public ModuleIOAlphabot(
            int driveCanID,
            int turnCanID,
            int cancoderCanID,
            double absoluteEncoderOffsetRad
    ) {
        zeroRotationRad = absoluteEncoderOffsetRad;
        driveSpark = new SparkMax(driveCanID, MotorType.kBrushless);
        turnSpark = new SparkMax(turnCanID, MotorType.kBrushless);
        cancoder = new CANcoder(cancoderCanID);
        driveEncoder = driveSpark.getEncoder();
        turnEncoder = turnSpark.getEncoder();
        driveController = driveSpark.getClosedLoopController();
        turnController = turnSpark.getClosedLoopController();

        // Configure drive motor
        driveConfig = new SparkMaxConfig();
        driveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(moduleConfig.driveCurrentLimit())
                .voltageCompensation(12.0);
        driveConfig
                .encoder
                .positionConversionFactor(2 * Math.PI / moduleConfig.driveGearRatio()) // Rotor Rotations -> Wheel Radians
                .velocityConversionFactor((2 * Math.PI) / 60.0 / moduleConfig.driveGearRatio()) // Rotor RPM -> Wheel Rad/Sec
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        driveConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        moduleConfig.driveGains().applySpark(driveConfig.closedLoop);
        driveConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.sparkFrequencyHz))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(5, () -> driveSpark.configure(
                driveConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        ));
        tryUntilOk(5, () -> driveEncoder.setPosition(0.0));

        // Configure turn motor
        turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(moduleConfig.turnInverted())
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(moduleConfig.turnCurrentLimit())
                .voltageCompensation(12.0);
        turnConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0.0, 2 * Math.PI);
        moduleConfig.turnGains().applySpark(turnConfig.closedLoop);
        turnConfig
                .signals
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(5, () -> turnSpark.configure(
                turnConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        ));

        // Configure CANCoder
        var cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.MagnetOffset = Units.radiansToRotations(absoluteEncoderOffsetRad);
        cancoderConfig.MagnetSensor.SensorDirection =
                moduleConfig.encoderInverted()
                        ? SensorDirectionValue.Clockwise_Positive
                        : SensorDirectionValue.CounterClockwise_Positive;
        cancoder.getConfigurator().apply(cancoderConfig);

        turnAbsolutePosition = cancoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, turnAbsolutePosition);

        // Reset turn spark
        // Not the prettiest but doing it now is better than in updateInputs
        var successful = false;
        // 15 attempts because this is really important
        for (int i = 0; i < 15; i++) {
            var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);
            var turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
            if (turnEncoderConnected) {
                var absolutePositionRad = Units.rotationsToRadians(turnAbsolutePosition.getValueAsDouble());
                if (absolutePositionRad != 0) {
                    sparkStickyFault = false;
                    tryUntilOk(5, () -> turnEncoder.setPosition(absolutePositionRad));
                    if (!sparkStickyFault) {
                        System.out.printf("Drive module with cancoder ID %d setting initial position of turn relative encoder to %s%n", cancoderCanID, absolutePositionRad);
                        successful = true;
                        break;
                    }
                }
            }
            System.out.printf("Drive module with cancoder ID %d FAILED on attempt %d to set initial position of turn relative encoder (connected: %s, sparkStickyFault: %s)%n", cancoderCanID, i + 1, turnEncoderConnected, sparkStickyFault);
        }
        if (!successful) {
            System.out.printf("Drive module with cancoder ID %d GAVE UP setting initial position of turn relative encoder%n", cancoderCanID);
            turnRelativeEncoderNotReset.set(true);
        }

        // Create odometry queues
        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
        turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Update drive inputs
        sparkStickyFault = false;
        ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
        ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
        ifOk(
                driveSpark,
                new DoubleSupplier[]{driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
                (values) -> inputs.driveAppliedVolts = values[0] * values[1]
        );
        ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
        inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

        // Update turn inputs
        sparkStickyFault = false;
        ifOk(
                turnSpark,
                turnEncoder::getPosition,
                (value) -> inputs.turnPositionRad = value - zeroRotationRad
        );
        ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
        ifOk(
                turnSpark,
                new DoubleSupplier[]{turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
                (values) -> inputs.turnAppliedVolts = values[0] * values[1]
        );
        ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        // Turn cancoder
        var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);
        inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
        inputs.turnAbsolutePositionRad = Units.rotationsToRadians(turnAbsolutePosition.getValueAsDouble());

        // Update odometry inputs
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositionsRad = turnPositionQueue.stream().mapToDouble((Double value) -> value - zeroRotationRad).toArray();
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        var newConfig = new SparkMaxConfig().idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
        tryUntilOkAsync(5, () -> driveSpark.configure(
                newConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters
        ));
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        var newConfig = new SparkMaxConfig().idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
        tryUntilOkAsync(5, () -> turnSpark.configure(
                newConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters
        ));
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveSpark.setVoltage(output);
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnSpark.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        var ffVolts = driveFF.calculate(velocityRadPerSec);
        driveController.setReference(
                velocityRadPerSec,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                ffVolts,
                ArbFFUnits.kVoltage
        );
    }

    @Override
    public void setTurnPosition(double positionRad) {
        double setpoint = MathUtil.inputModulus(positionRad + zeroRotationRad, 0.0, 2 * Math.PI);
        turnController.setReference(setpoint, ControlType.kPosition);
    }
}
