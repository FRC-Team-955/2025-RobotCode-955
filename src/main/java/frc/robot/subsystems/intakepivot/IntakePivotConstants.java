package frc.robot.subsystems.intakepivot;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.PIDF;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOSparkMax;
import frc.robot.BuildConstants;

public class IntakePivotConstants {
    static final double gearRatio = 120;
    static final PIDF gains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> PIDF.ofPG(0.5, 0);
        case SIM -> PIDF.ofPDG(16.7, 0.01, 2.68);
    };

    static MotorIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new MotorIOSparkMax(
                    6,
                    false,
                    true,
                    40,
                    gearRatio,
                    gains,
                    PIDF.ofP(0)
            );
            case SIM -> new IntakePivotIOSim(
                    gearRatio,
                    DCMotor.getNEO(1),
                    gains
            );
            case REPLAY -> new MotorIO();
        };
    }
}
