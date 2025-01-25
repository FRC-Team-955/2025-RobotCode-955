package frc.robot.subsystems.drive;

import com.ctre.phoenix6.CANBus;
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

    public static final String canbusName = "phoenix";
    public static final boolean isCANFD = switch (Constants.identity) {
        case COMPBOT -> new CANBus(canbusName).isNetworkFD();
        case ALPHABOT, SIMBOT -> false;
    };

    public static final DriveConfig driveConfig = switch (Constants.identity) {
        case COMPBOT, SIMBOT -> new DriveConfig(
                Units.inchesToMeters(2),
                Units.inchesToMeters(21.75),
                Units.inchesToMeters(21.75),
                Units.inchesToMeters(31),
                Units.inchesToMeters(31),
                // from Choreo
                4.731,
                12.960,
                12.112,
                40.186,
                PIDF.ofPD(1.5, 0),
                PIDF.ofPD(1.5, 0)
        );
        case ALPHABOT -> new DriveConfig(
                Units.inchesToMeters(2),
                Units.inchesToMeters(20.75),
                Units.inchesToMeters(20.75),
                Units.inchesToMeters(30),
                Units.inchesToMeters(30),
                // from Choreo
                4.637,
                12.123,
                12.442,
                35.864,
                PIDF.ofPD(1.5, 0),
                PIDF.ofPD(1.5, 0)
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

    public static final ModuleConfig moduleConfig = switch (Constants.identity) {
        case COMPBOT -> new ModuleConfig(
                PIDF.ofPDSVA(
                        0.1, 0.0,
                        // FL + FR + BL + BR
                        Util.average(0.21524, 0.16554, 0.083665, 0.061984),
                        Util.average(0.11224, 0.11693, 0.12106, 0.12449),
                        Util.average(0.0038991, 0.0018671, 0.004602, 0.0059919)
                ),
                PIDF.ofPDSVA(100.0, 0.0, 0.1, 1.91, 0),
                Mk4iGearRatios.L2,
                Mk4iGearRatios.TURN,
                true,
                false,
                false,
                120,
                60
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
                50,
                20
        );
        case SIMBOT -> new ModuleConfig(
                PIDF.ofPDSV(0.1, 0.0, 0.0, 0.13),
                PIDF.ofPD(10.0, 0.0),
                Mk4iGearRatios.L2,
                Mk4iGearRatios.TURN,
                true,
                false,
                false,
                0,
                0
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
                new ModuleIOCompbot(10, 12, 13, 0.189),
                new ModuleIOCompbot(7, 8, 9, 1.891),
                new ModuleIOCompbot(1, 2, 3, -0.009),
                new ModuleIOCompbot(5, 6, 4, 1.525),
        };
        case ALPHABOT -> new ModuleIO[]{
                // FL, FR, BL, BR
                new ModuleIOAlphabot(4, 5, 6, -2.115),
                new ModuleIOAlphabot(2, 3, 1, -2.161),
                new ModuleIOAlphabot(9, 10, 8, 0.255),
                new ModuleIOAlphabot(12, 13, 11, 0.852),
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
        case COMPBOT -> new GyroIOPigeon2(11);
        case ALPHABOT -> new GyroIOPigeon2(7);
        case SIMBOT -> new GyroIO();
    };

    public static final double drivebaseRadius = Math.hypot(driveConfig.trackWidthMeters / 2.0, driveConfig.trackLengthMeters / 2.0);

    public static final double joystickMaxAngularSpeedRadPerSec = Math.min(Units.degreesToRadians(315), driveConfig.maxAngularSpeedRadPerSec());
    public static final double joystickDriveDeadband = 0.1;

    public static final double assistDirectionToleranceRad = Units.degreesToRadians(50);
    public static final double assistMaximumDistanceMeters = Units.feetToMeters(5);

    public static final PIDF moveToXY = PIDF.ofPD(4, 0);
    public static final PIDF moveToOmega = PIDF.ofPD(1.5, 0);

    public record DriveConfig(
            double wheelRadiusMeters,
            double trackWidthMeters, // Measured from the center of the swerve wheels
            double trackLengthMeters,
            double bumperWidthMeters,
            double bumperLengthMeters,
            double maxLinearSpeedMetersPerSec,
            double maxLinearAccelMetersPerSecSquared,
            double maxAngularSpeedRadPerSec,
            double maxAngularAccelRadPerSecSquared,
            PIDF choreoFeedbackXY,
            PIDF choreoFeedbackOmega
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
