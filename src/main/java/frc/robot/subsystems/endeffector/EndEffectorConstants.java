package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.BuildConstants;
import frc.robot.subsystems.rollers.RollersConfig;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOSim;
import frc.robot.subsystems.rollers.RollersIOSparkMax;
import frc.robot.util.PIDF;

public class EndEffectorConstants {
    public static final double rollersPositionToleranceRad = Units.degreesToRadians(15);
    public static final double rollersRadiusMeters = Units.inchesToMeters(2.25 / 2.0);

    public static final double descoreAlgaeTriggerAmps = 27;

    public static final double extendStartMeters = Units.inchesToMeters(5);
    public static final double extendDistanceMeters = Units.inchesToMeters(2.25);
    public static final double angleWhenExtendedRad = Units.degreesToRadians(40);
    public static final double angleWhenRetractedRad = Units.degreesToRadians(90);

    public static double rollersRadiansForMeters(double meters) {
        return meters / rollersRadiusMeters;
    }

    public static final RollersConfig rollersConfig = new RollersConfig(
            false,
            true,
            40,
            9,
            switch (BuildConstants.mode) {
                case REAL, REPLAY -> PIDF.ofP(0.5);
                case SIM -> PIDF.ofP(3);
            },
            switch (BuildConstants.mode) {
                case REAL, REPLAY -> PIDF.ofPSV(0.01, 0.42461, 0.18272);
                case SIM -> PIDF.ofSV(0.00995, 0.17859);
            }
    );

    protected static RollersIO createRollersIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new RollersIOSparkMax(6, rollersConfig);
            case SIM -> new RollersIOSim(
                    rollersConfig,
                    0.01,
                    DCMotor.getNEO(1)
            );
            case REPLAY -> new RollersIO();
        };
    }
}