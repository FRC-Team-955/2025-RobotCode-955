package frc.lib.motor;

import frc.lib.motor.MotorIO.MotorIOInputs;
import lombok.RequiredArgsConstructor;

import java.util.function.BiConsumer;

@RequiredArgsConstructor
public class RequestHelper {
    // Get a reference to the inputs so we don't need to take them as a parameter
    private final MotorIOInputs inputs;
    private final RequestTolerances tolerances;

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
                handleRelativePositionRequest(goalHash, type, value)
        );
    }

    private int relativePositionLastGoalHash = 0;
    private double relativePositionLastGoalValue = 0.0;
    private double relativePositionAbsoluteSetpoint = 0.0;

    private double handleRelativePositionRequest(
            int goalHash,
            RequestType type,
            double value
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

    public boolean atRequest(RequestType type, double value) {
        return switch (type) {
            case PositionRad -> Math.abs(inputs.positionRad - value) <= tolerances.positionToleranceRad();

            case RelativePositionRad -> {
                if (relativePositionLastGoalHash == 0) {
                    // relativePositionAbsoluteSetpoint is not set or invalid
                    yield false;
                } else {
                    yield Math.abs(inputs.positionRad - relativePositionAbsoluteSetpoint) <= tolerances.positionToleranceRad();
                }
            }

            case VelocityRadPerSec ->
                    Math.abs(inputs.velocityRadPerSec - value) <= tolerances.velocityToleranceRadPerSec();

            case VoltageVolts -> Math.abs(inputs.appliedVolts - value) <= tolerances.voltageToleranceVolts();
        };
    }
}
