package frc.lib.motor;

import frc.lib.motor.MotorIO.MotorIOInputs;
import lombok.RequiredArgsConstructor;

import java.util.function.BiConsumer;

@RequiredArgsConstructor
public class RequestConverter {
    // Get a reference to the inputs so we don't need to take them as a parameter
    private final MotorIOInputs inputs;

    public void convertAndApplyRequest(
            int goalHash,
            RequestType type,
            double value,
            BiConsumer<MotorIO.RequestType, Double> setAndLogRequest
    ) {
        setAndLogRequest.accept(
                switch (type) {
                    case VoltageVolts -> MotorIO.RequestType.VoltageVolts;
                    case PositionRad, RelativePositionRad -> MotorIO.RequestType.PositionRad;
                    case VelocityRadPerSec -> MotorIO.RequestType.VelocityRadPerSec;
                },
                handleRelativePositionRequest(goalHash, type, value, inputs)
        );
    }

    private int relativePositionLastGoalHash = 0;
    private double relativePositionLastGoalValue = 0.0;
    private double relativePositionAbsoluteSetpoint = 0.0;

    private double handleRelativePositionRequest(
            int goalHash,
            RequestType type,
            double value,
            MotorIOInputs inputs
    ) {
        if (type == RequestType.RelativePositionRad) {
            // Reset if new goal or new relative setpoint
            if (goalHash != relativePositionLastGoalHash || value != relativePositionLastGoalValue) {
                relativePositionLastGoalHash = goalHash;
                relativePositionLastGoalValue = value;

                relativePositionAbsoluteSetpoint = inputs.positionRad + value;
            }

            return relativePositionAbsoluteSetpoint;
        } else {
            // Invalid current setpoint and ensure that a new setpoint is generated if the type goes back to relative position
            // hashCode of null is 0 - if the goal hash is 0, something already went horribly wrong
            relativePositionLastGoalHash = 0;
            relativePositionLastGoalValue = 0;

            return value;
        }
    }
}
