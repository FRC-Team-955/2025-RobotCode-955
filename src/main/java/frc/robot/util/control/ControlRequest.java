package frc.robot.util.control;

import java.util.function.DoubleConsumer;

public class ControlRequest {
    private enum Type {
        VOLTAGE,
        POSITION,
        VELOCITY
    }

    private final Type type;
    private final double value;

    private ControlRequest(Type type, double value) {
        this.type = type;
        this.value = value;
    }

    public static ControlRequest voltage(double volts) {
        return new ControlRequest(Type.VOLTAGE, volts);
    }

    public static ControlRequest position(double positionRad) {
        return new ControlRequest(Type.POSITION, positionRad);
    }

    public static ControlRequest velocity(double velocityRadPerSec) {
        return new ControlRequest(Type.VELOCITY, velocityRadPerSec);
    }

    public void handle(
            DoubleConsumer voltageConsumer,
            DoubleConsumer positionRadConsumer,
            DoubleConsumer velocityRadPerSecConsumer
    ) {
        switch (type) {
            case VOLTAGE -> voltageConsumer.accept(value);
            case POSITION -> positionRadConsumer.accept(value);
            case VELOCITY -> velocityRadPerSecConsumer.accept(value);
        }
    }
}
