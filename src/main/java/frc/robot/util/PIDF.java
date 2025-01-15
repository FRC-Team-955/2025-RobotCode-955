package frc.robot.util;

import com.ctre.phoenix6.configs.Slot0Configs;

public record PIDF(double p, double i, double d, double s, double v, double a, double g) {
    public static PIDF ofP(double p) {
        return new PIDF(p, 0, 0, 0, 0, 0, 0);
    }

    public static PIDF ofPD(double p, double d) {
        return new PIDF(p, 0, d, 0, 0, 0, 0);
    }

    public static PIDF ofPID(double p, double i, double d) {
        return new PIDF(p, i, d, 0, 0, 0, 0);
    }

    public static PIDF ofPSV(double p, double s, double v) {
        return new PIDF(p, 0, 0, s, v, 0, 0);
    }

    public static PIDF ofPDS(double p, double d, double s) {
        return new PIDF(p, 0, d, s, 0, 0, 0);
    }

    public static PIDF ofPDSV(double p, double d, double s, double v) {
        return new PIDF(p, 0, d, s, v, 0, 0);
    }

    public static PIDF ofPDSVA(double p, double d, double s, double v, double a) {
        return new PIDF(p, 0, d, s, v, a, 0);
    }

    public static PIDF ofPIDSVA(double p, double i, double d, double s, double v, double a) {
        return new PIDF(p, i, d, s, v, a, 0);
    }

    public static PIDF ofPIDSVAG(double p, double i, double d, double s, double v, double a, double g) {
        return new PIDF(p, i, d, s, v, a, g);
    }

    public Slot0Configs toPhoenix() {
        return new Slot0Configs()
                .withKP(p)
                .withKI(i)
                .withKD(d)
                .withKS(s)
                .withKV(v)
                .withKA(a)
                .withKG(g);
    }
}
