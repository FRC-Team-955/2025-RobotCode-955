package frc.robot.util.control;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

public class SwerveControlRequest {
    private enum Type {
        VOLTAGE,
        CHASSIS_SPEEDS
    }

    private final Type type;
    private double volts;
    private ChassisSpeeds chassisSpeeds;

    private SwerveControlRequest(Type type, double volts) {
        this.type = type;
        this.volts = volts;
    }

    private SwerveControlRequest(Type type, ChassisSpeeds chassisSpeeds) {
        this.type = type;
        this.chassisSpeeds = chassisSpeeds;
    }

    public static SwerveControlRequest voltage(double volts) {
        return new SwerveControlRequest(Type.VOLTAGE, volts);
    }

    public static SwerveControlRequest chassisSpeeds(ChassisSpeeds chassisSpeeds) {
        return new SwerveControlRequest(Type.CHASSIS_SPEEDS, chassisSpeeds);
    }

    public void handle(
            DoubleConsumer voltageConsumer,
            Consumer<ChassisSpeeds> chassisSpeedsConsumer
    ) {
        switch (type) {
            case VOLTAGE -> voltageConsumer.accept(volts);
            case CHASSIS_SPEEDS -> chassisSpeedsConsumer.accept(chassisSpeeds);
        }
    }
}
