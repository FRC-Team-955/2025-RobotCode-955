package frc.robot.util;

public record PIDF(double p, double i, double d, double s, double v, double a) {
    public PIDF(double p, double s, double v) {
        this(p, 0, 0, s, v, 0);
    }

    public PIDF(double p, double d, double s, double v) {
        this(p, 0, d, s, v, 0);
    }

    public PIDF(double p, double d, double s, double v, double a) {
        this(p, 0, d, s, v, a);
    }
}
