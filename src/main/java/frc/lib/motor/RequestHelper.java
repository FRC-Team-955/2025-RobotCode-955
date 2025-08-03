package frc.lib.motor;

import frc.lib.motor.MotorIO.MotorIOInputs;
import lombok.RequiredArgsConstructor;

import java.util.function.BiConsumer;

@RequiredArgsConstructor
public class RequestHelper {
    // Get a reference to the inputs so we don't need to take them as a parameter
    private final MotorIOInputs inputs;
    private final RequestTolerances tolerances;

    private Integer lastGoalHash = null;
    private RequestType lastType = null;
    private Double lastValue = null;

    private boolean atLastGoal = false;

    /** Should be called in periodicBeforeCommands */
    public boolean processAtLastGoal() {
        if (lastGoalHash == null || lastType == null || lastValue == null) {
            return false;
        }

        atLastGoal = switch (lastType) {
            case PositionRad -> Math.abs(inputs.positionRad - lastValue) <= tolerances.positionToleranceRad();

            case RelativePositionRad -> {
                if (relativePositionLastGoalHash == 0) {
                    // relativePositionAbsoluteSetpoint is not set or invalid
                    yield false;
                } else {
                    yield Math.abs(inputs.positionRad - relativePositionAbsoluteSetpoint) <= tolerances.positionToleranceRad();
                }
            }

            case VelocityRadPerSec ->
                    Math.abs(inputs.velocityRadPerSec - lastValue) <= tolerances.velocityToleranceRadPerSec();

            case VoltageVolts -> Math.abs(inputs.appliedVolts - lastValue) <= tolerances.voltageToleranceVolts();
        };
        return atLastGoal;
    }

    /** Should be called by commands */
    public boolean isAtLastGoal(int goalHash) {
        if (lastGoalHash == null || goalHash != lastGoalHash) {
            // Wait for processAtLastGoal to be called
            // We don't want a false positive by looking at the previous goal
            return false;
        }

        return atLastGoal;
    }

    /** Should be called in periodicAfterCommands */
    public void convertAndApplyRequest(
            int goalHash,
            RequestType type,
            double value,
            BiConsumer<MotorIO.RequestType, Double> setAndLogRequest
    ) {
        lastGoalHash = goalHash;
        lastType = type;
        lastValue = value;

        setAndLogRequest.accept(
                switch (type) {
                    case VoltageVolts -> MotorIO.RequestType.VoltageVolts;
                    case PositionRad, RelativePositionRad -> MotorIO.RequestType.PositionRad;
                    case VelocityRadPerSec -> MotorIO.RequestType.VelocityRadPerSec;
                },
                handleRelativePositionRequest()
        );
    }

    private int relativePositionLastGoalHash = 0;
    private double relativePositionLastValue = 0.0;
    private double relativePositionAbsoluteSetpoint = 0.0;

    private double handleRelativePositionRequest() {
        if (lastType == RequestType.RelativePositionRad) {
            // Reset if new goal or new relative setpoint
            if (lastGoalHash != relativePositionLastGoalHash || lastValue != relativePositionLastValue) {
                relativePositionLastGoalHash = lastGoalHash;
                relativePositionLastValue = lastValue;

                relativePositionAbsoluteSetpoint = inputs.positionRad + lastValue;
            }

            return relativePositionAbsoluteSetpoint;
        } else {
            // Invalid current setpoint and ensure that a new setpoint is generated if the type goes back to relative position
            // hashCode of null is 0 - if the goal hash is 0, something already went horribly wrong
            relativePositionLastGoalHash = 0;
            relativePositionLastValue = 0;

            return lastValue;
        }
    }
}
