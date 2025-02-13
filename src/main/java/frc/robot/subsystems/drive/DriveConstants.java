package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Util;
import frc.robot.util.PIDF;

public class DriveConstants {
    public static final double phoenixFrequencyHz = switch (Constants.identity) {
        case COMPBOT, SIMBOT -> 250.0;
        case ALPHABOT -> 100.0;
    };
    public static final double sparkFrequencyHz = switch (Constants.identity) {
        case COMPBOT, SIMBOT -> 250.0;
        case ALPHABOT -> 100.0;
    };

    public static final double assistDirectionToleranceRad = Units.degreesToRadians(50);
    public static final double assistMaximumDistanceMeters = Units.feetToMeters(5);

    // Ranges: -1 to 1, where 1 is the maximum speed
    public static final PIDF moveToXY = PIDF.ofPD(2.7, 0.2);
    public static final PIDF moveToOmega = PIDF.ofPD(1.5, 0);

    public static final boolean useSetpointGenerator = true;
    public static final boolean disableDriving = false;

    // Slow to 30% speed when elevator is at max height
    public static final double elevatorSlowdownScalar = 0.7;

    public static final DriveConfig driveConfig = switch (Constants.identity) {
        case COMPBOT, SIMBOT -> new DriveConfig(
                Units.inchesToMeters(2),
                Units.inchesToMeters(22.75),
                Units.inchesToMeters(22.75),
                Units.inchesToMeters(34.625),
                Units.inchesToMeters(34.625),
                PIDF.ofPD(1.5, 0),
                PIDF.ofPD(1.5, 0),
                4.574, // from Choreo
                20,
                11.580, // from Choreo
                55.116, // from Choreo
                Units.degreesToRadians(1080)
        );
        case ALPHABOT -> new DriveConfig(
                Units.inchesToMeters(2),
                Units.inchesToMeters(20.75),
                Units.inchesToMeters(20.75),
                Units.inchesToMeters(30),
                Units.inchesToMeters(30),
                PIDF.ofPD(1.5, 0),
                PIDF.ofPD(1.5, 0),
                4.637, // from Choreo
                20,
                12.442, // from Choreo
                31.923, // from Choreo
                Units.degreesToRadians(1080) // not from Choreo
        );
    };

    /**
     * FL, FR, BL, BR
     */
    public static final Translation2d[] moduleTranslations = new Translation2d[]{
            new Translation2d(driveConfig.trackWidthMeters / 2.0, driveConfig.trackLengthMeters / 2.0),
            new Translation2d(driveConfig.trackWidthMeters / 2.0, -driveConfig.trackLengthMeters / 2.0),
            new Translation2d(-driveConfig.trackWidthMeters / 2.0, driveConfig.trackLengthMeters / 2.0),
            new Translation2d(-driveConfig.trackWidthMeters / 2.0, -driveConfig.trackLengthMeters / 2.0)
    };

    public static final double drivebaseRadius = Math.hypot(driveConfig.trackWidthMeters / 2.0, driveConfig.trackLengthMeters / 2.0);

    public static final double joystickMaxAngularSpeedRadPerSec = Math.min(Units.degreesToRadians(315), driveConfig.maxAngularSpeedRadPerSec());
    public static final double joystickDriveDeadband = 0.1;

    public static final ModuleConfig moduleConfig = switch (Constants.identity) {
        case COMPBOT -> new ModuleConfig(
                PIDF.ofPDSV(
                        0.0, 0.0,
                        // TODO: units are messed up
                        0.14, 0.12 // velocity - units are 12 / radPerSec, so multiply by 12
                ),
                PIDF.ofPD(5, 0.0),
                Mk4iGearRatios.L2,
                Mk4iGearRatios.TURN,
                true,
                false,
                false,
                120,
                20
        );
        case ALPHABOT -> new ModuleConfig(
                PIDF.ofPDSVA(
                        0.0, 0.0,
                        // FL + FR + BL + BR
                        Util.average(0.024319, 0.094701 /* , [erroneous], [erroneous] */),
                        Util.average(0.13551, 0.13733, 0.13543, 0.14087),
                        Util.average(0.0065694, 0.0054738, /* [erroneous], */ 0.0091241)
                ),
                PIDF.ofPD(0.5, 0.0),
                Mk4iGearRatios.L2,
                Mk4iGearRatios.TURN,
                true,
                false,
                false,
                60,
                20
        );
        case SIMBOT -> new ModuleConfig(
                PIDF.ofPDSV(0.05, 0.0, 0.0, 0.15),
                PIDF.ofPD(8.0, 0.0),
                Mk4iGearRatios.L2,
                Mk4iGearRatios.TURN,
                true,
                false,
                false,
                120,
                20
        );
    };

    // IO layers should go at the bottom in case they reference constants that aren't yet initialized

    public static final ModuleIO[] moduleIO = Constants.isReplay
            ? new ModuleIO[]{new ModuleIO(), new ModuleIO(), new ModuleIO(), new ModuleIO()}
            : switch (Constants.identity) {
        // To calibrate the absolute encoder offsets, point the modules straight (such that forward
        // motion on the drive motor will propel the robot forward) and copy the reported values from the
        // absolute encoders using AdvantageScope. These values are logged under "/Inputs/Drive/ModuleX/TurnAbsolutePositionRad"
        case COMPBOT -> new ModuleIO[]{
                // FL, FR, BL, BR
                new ModuleIOTalonFXSparkMaxCANcoder(1, 1, 5, 1.577),
                new ModuleIOTalonFXSparkMaxCANcoder(2, 2, 6, 1.770),
                new ModuleIOTalonFXSparkMaxCANcoder(3, 3, 7, 3.105),
                new ModuleIOTalonFXSparkMaxCANcoder(4, 4, 8, -2.817),
        };
        case ALPHABOT -> new ModuleIO[]{
                // FL, FR, BL, BR
                new ModuleIOSparkMaxCANcoder(4, 5, 6, -2.115),
                new ModuleIOSparkMaxCANcoder(2, 3, 1, -2.161),
                new ModuleIOSparkMaxCANcoder(9, 10, 8, 0.255),
                new ModuleIOSparkMaxCANcoder(12, 13, 11, 0.852),
        };
        case SIMBOT -> new ModuleIO[]{
                new ModuleIOSim(0),
                new ModuleIOSim(1),
                new ModuleIOSim(2),
                new ModuleIOSim(3)
        };
    };

    public static final GyroIO gyroIO = Constants.isReplay
            ? new GyroIO()
            : switch (Constants.identity) {
        case COMPBOT -> new GyroIOPigeon2(9);
        case ALPHABOT -> new GyroIOPigeon2(7);
        case SIMBOT -> new GyroIOSim();
    };

    public record DriveConfig(
            double wheelRadiusMeters,
            double trackWidthMeters, // Measured from the center of the swerve wheels
            double trackLengthMeters,
            double bumperWidthMeters,
            double bumperLengthMeters,
            PIDF choreoFeedbackXY,
            PIDF choreoFeedbackOmega,
            double maxLinearSpeedMetersPerSec,
            double maxLinearAccelMetersPerSecSquared,
            double maxAngularSpeedRadPerSec,
            double maxAngularAccelRadPerSecSquared,
            double maxTurnVelocityRadPerSec // Maximum velocity of the turn motor
    ) {
    }

    public record ModuleConfig(
            PIDF driveGains,
            PIDF turnGains,
            double driveGearRatio,
            double turnGearRatio,
            boolean turnInverted,
            boolean driveInverted,
            boolean encoderInverted,
            int driveCurrentLimit, // AKA current that causes wheel slip
            int turnCurrentLimit
    ) {
    }

    private static class Mk4iGearRatios {
        public static final double L2 = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
        public static final double L3 = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);

        public static final double TURN = (150.0 / 7.0);
    }
}
