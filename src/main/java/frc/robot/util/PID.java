package frc.robot.util;

public record PID(double p, double i, double d) {
    public PID(double p) {
        this(p, 0, 0);
    }

    public PID(double p, double d) {
        this(p, 0, d);
    }
}
