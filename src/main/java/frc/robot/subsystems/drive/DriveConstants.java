package frc.robot.subsystems.drive;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class DriveConstants {
    public static final DriveConfig driveConfig = switch (Constants.identity) {
        case COMPBOT, SIMBOT -> new DriveConfig(
                Units.inchesToMeters(1.90433686321622),
                Units.inchesToMeters(21.75),
                Units.inchesToMeters(21.75),
                Units.inchesToMeters(25),
                Units.inchesToMeters(25),
                new PIDConstants(1.5, 0, 0),
                new PIDConstants(1.5, 0, 0),
                new PIDConstants(2.1, 0, 0.1)
        );
        case NEO_DRIVEBASE -> new DriveConfig(
                Units.inchesToMeters(2),
                Units.inchesToMeters(21.75),
                Units.inchesToMeters(21.75),
                Units.inchesToMeters(25),
                Units.inchesToMeters(25),
                new PIDConstants(1.5, 0, 0),
                new PIDConstants(1.5, 0, 0),
                new PIDConstants(2.1, 0, 0.1)
        );
    };

    public static final double drivebaseRadius = Math.hypot(driveConfig.trackWidthMeters / 2.0, driveConfig.trackLengthMeters / 2.0);

    /**
     * FL, FR, BL, BR
     */
    public static final Translation2d[] moduleTranslations = new Translation2d[]{
            new Translation2d(driveConfig.trackWidthMeters / 2.0, driveConfig.trackLengthMeters / 2.0),
            new Translation2d(driveConfig.trackWidthMeters / 2.0, -driveConfig.trackLengthMeters / 2.0),
            new Translation2d(-driveConfig.trackWidthMeters / 2.0, driveConfig.trackLengthMeters / 2.0),
            new Translation2d(-driveConfig.trackWidthMeters / 2.0, -driveConfig.trackLengthMeters / 2.0)
    };

    public static final ModuleConfig moduleConfig = switch (Constants.identity) {
        case COMPBOT -> new ModuleConfig(
                new SimpleMotorFeedforward(0.0689045, 0.1370225, 0.009731425),
                new PIDConstants(0.05, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0),
                Mk4iGearRatios.L2,
                Mk4iGearRatios.TURN
        );
        case NEO_DRIVEBASE -> new ModuleConfig(
                new SimpleMotorFeedforward(0.0689045, 0.1370225, 0.009731425),
                new PIDConstants(0.05, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0),
                Mk4iGearRatios.L2,
                Mk4iGearRatios.TURN
        );
        case SIMBOT -> new ModuleConfig(
                new SimpleMotorFeedforward(0.0, 0.13),
                new PIDConstants(0.1, 0.0, 0.0),
                new PIDConstants(10.0, 0.0, 0.0),
                Mk4iGearRatios.L2,
                Mk4iGearRatios.TURN
        );
    };

    public static final ModuleIO[] moduleIO = Constants.isReplay
            ? new ModuleIO[]{new ModuleIO(), new ModuleIO(), new ModuleIO(), new ModuleIO()}
            : switch (Constants.identity) {
        // To calibrate the absolute encoder offsets, point the modules straight (such that forward
        // motion on the drive motor will propel the robot forward) and copy the reported values from the
        // absolute encoders using AdvantageScope. These values are logged under "/Inputs/Drive/ModuleX/TurnAbsolutePositionRad"
        case COMPBOT -> new ModuleIO[]{
                // FL, FR, BL, BR
                new ModuleIOTalonFXSparkMaxCANcoder(10, 12, 13, 0.0),
                new ModuleIOTalonFXSparkMaxCANcoder(7, 8, 9, 0.0),
                new ModuleIOTalonFXSparkMaxCANcoder(1, 2, 3, 0.0),
                new ModuleIOTalonFXSparkMaxCANcoder(5, 6, 4, 0.0),
        };
        case NEO_DRIVEBASE -> new ModuleIO[]{
                // FL, FR, BL, BR
                new ModuleIOSparkMaxCANcoder(2, 3, 1, -3.100),
                new ModuleIOSparkMaxCANcoder(12, 13, 11, 0.0),
                new ModuleIOSparkMaxCANcoder(4, 5, 6, -0.147),
                new ModuleIOSparkMaxCANcoder(9, 10, 8, 0.0),
        };
        case SIMBOT -> new ModuleIO[]{
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim()
        };
    };

    public static final GyroIO gyroIO = Constants.isReplay
            ? new GyroIO()
            : switch (Constants.identity) {
        case COMPBOT -> new GyroIOPigeon2(0);
        case NEO_DRIVEBASE -> new GyroIOPigeon2(0);
        case SIMBOT -> new GyroIO();
    };

    public record DriveConfig(
            double wheelRadius,
            double trackWidthMeters, // Measured from the center of the swerve wheels
            double trackLengthMeters,
            double bumperWidthMeters,
            double bumperLengthMeters,
            PIDConstants choreoFeedbackXY,
            PIDConstants choreoFeedbackTheta,
            PIDConstants pointTowardsController
    ) {
    }

    public record ModuleConfig(
            SimpleMotorFeedforward driveFeedforward,
            PIDConstants driveFeedback,
            PIDConstants turnFeedback,
            double driveGearRatio,
            double turnGearRatio
    ) {
    }


    private static class Mk4iGearRatios {
        public static final double L2 = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
        public static final double L3 = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);

        public static final double TURN = (150.0 / 7.0);
    }
}
