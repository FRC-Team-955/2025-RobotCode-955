package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

class SuperstructureConstants {
    public static final double scoreCoralSettleSeconds = 0.25;
    public static final double scoreCoralL1SettleSeconds = 0.5;

    public static final Transform3d coralAboveFunnel = new Transform3d(
            Units.inchesToMeters(5),
            0,
            Units.inchesToMeters(18),
            new Rotation3d(0, -Units.degreesToRadians(45), 0)
    );
    public static final Transform3d coralInFunnel = new Transform3d(
            Units.inchesToMeters(2),
            0,
            Units.inchesToMeters(12),
            new Rotation3d()
    );

    public static Transform3d coralInEndEffector(double elevatorPositionMeters, double endEffectorAngleRad) {
        endEffectorAngleRad += Math.PI / 2.0;
        double tan = Math.tan(endEffectorAngleRad);
        return new Transform3d(
                Units.inchesToMeters(-10) + Units.inchesToMeters(3) * tan,
                0,
                Units.inchesToMeters(13.5) + elevatorPositionMeters + Units.inchesToMeters(1) * tan,
                new Rotation3d(0, endEffectorAngleRad, 0)
        );
    }

    protected static SuperstructureIO createIO() {
        if (Constants.isReplay) {
            return new SuperstructureIO();
        }
        return switch (Constants.identity) {
            case COMPBOT -> new SuperstructureIOReal();
            case SIMBOT -> new SuperstructureIOSim();
            case ALPHABOT -> new SuperstructureIO();
        };
    }
}