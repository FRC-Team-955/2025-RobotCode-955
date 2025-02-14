package frc.robot.util;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public record PIDF(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    public static PIDF ofP(double kP) {
        return new PIDF(kP, 0, 0, 0, 0, 0, 0);
    }

    public static PIDF ofPD(double kP, double kD) {
        return new PIDF(kP, 0, kD, 0, 0, 0, 0);
    }

    public static PIDF ofPID(double kP, double kI, double kD) {
        return new PIDF(kP, kI, kD, 0, 0, 0, 0);
    }

    public static PIDF ofPSV(double kP, double kS, double kV) {
        return new PIDF(kP, 0, 0, kS, kV, 0, 0);
    }

    public static PIDF ofPSVG(double kP, double kS, double kV, double kG) {
        return new PIDF(kP, 0, 0, kS, kV, 0, kG);
    }

    public static PIDF ofPSVA(double kP, double kS, double kV, double kA) {
        return new PIDF(kP, 0, 0, kS, kV, kA, 0);
    }

    public static PIDF ofPSVAG(double kP, double kS, double kV, double kA, double kG) {
        return new PIDF(kP, 0, 0, kS, kV, kA, kG);
    }

    public static PIDF ofPDS(double kP, double kD, double kS) {
        return new PIDF(kP, 0, kD, kS, 0, 0, 0);
    }

    public static PIDF ofPDSV(double kP, double kD, double kS, double kV) {
        return new PIDF(kP, 0, kD, kS, kV, 0, 0);
    }

    public static PIDF ofPDSVG(double kP, double kD, double kS, double kV, double kG) {
        return new PIDF(kP, 0, kD, kS, kV, 0, kG);
    }

    public static PIDF ofPDSG(double kP, double kD, double kS, double kG) {
        return new PIDF(kP, 0, kD, kS, 0, 0, kG);
    }

    public static PIDF ofPDSVA(double kP, double kD, double kS, double kV, double kA) {
        return new PIDF(kP, 0, kD, kS, kV, kA, 0);
    }

    public static PIDF ofPDSVAG(double kP, double kD, double kS, double kV, double kA, double kG) {
        return new PIDF(kP, 0, kD, kS, kV, kA, kG);
    }

    public static PIDF ofPIDSVA(double kP, double kI, double kD, double kS, double kV, double kA) {
        return new PIDF(kP, kI, kD, kS, kV, kA, 0);
    }

    public static PIDF ofPIDSVAG(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
        return new PIDF(kP, kI, kD, kS, kV, kA, kG);
    }

    public void applySpark(ClosedLoopConfig config, ClosedLoopSlot slot) {
        config.pidf(kP, kI, kD, 0, slot);
    }

    public SlotConfigs toPhoenix() {
        return new SlotConfigs()
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKS(kS)
                .withKV(kV)
                .withKA(kA);
    }

    public SlotConfigs toPhoenix(StaticFeedforwardSignValue staticFeedforwardSign) {
        return new SlotConfigs()
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKS(kS)
                .withKV(kV)
                .withKA(kA)
                .withStaticFeedforwardSign(staticFeedforwardSign);
    }

    public SlotConfigs toPhoenixWithGravity(GravityTypeValue gravityType) {
        return new SlotConfigs()
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKS(kS)
                .withKV(kV)
                .withKA(kA)
                .withKG(kG)
                .withGravityType(gravityType);
    }

    public SlotConfigs toPhoenixWithGravity(GravityTypeValue gravityType, StaticFeedforwardSignValue staticFeedforwardSign) {
        return new SlotConfigs()
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKS(kS)
                .withKV(kV)
                .withKA(kA)
                .withKG(kG)
                .withGravityType(gravityType)
                .withStaticFeedforwardSign(staticFeedforwardSign);
    }

    public PIDController toPID() {
        return new PIDController(kP, kI, kD);
    }

    public PIDController toPIDWrapRadians() {
        var pid = new PIDController(kP, kI, kD);
        pid.enableContinuousInput(-Math.PI, Math.PI);
        return pid;
    }

    public ProfiledPIDController toProfiledPID(TrapezoidProfile.Constraints constraints) {
        return new ProfiledPIDController(kP, kI, kD, constraints);
    }

    public ProfiledPIDController toProfiledPIDWrapRadians(TrapezoidProfile.Constraints constraints) {
        var pid = new ProfiledPIDController(kP, kI, kD, constraints);
        pid.enableContinuousInput(-Math.PI, Math.PI);
        return pid;
    }

    public SimpleMotorFeedforward toSimpleFF() {
        return new SimpleMotorFeedforward(kS, kV, kA);
    }

    public ArmFeedforward toArmFF() {
        return new ArmFeedforward(kS, kG, kV, kA);
    }

    public ElevatorFeedforward toElevatorFF() {
        return new ElevatorFeedforward(kS, kG, kV, kA);
    }
}
