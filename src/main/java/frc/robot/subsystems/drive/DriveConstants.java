package frc.robot.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Util;
import frc.robot.generated.TunerConstants;
import frc.robot.util.PID;
import frc.robot.util.PIDF;

public class DriveConstants {
    public static final double phoenixFrequency = 250.0;
    public static final double sparkFrequency = 100.0;

    public static final boolean isCANFD = new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD();

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
                new PID(1.5, 0, 0),
                new PID(1.5, 0, 0),
                new PID(2.1, 0, 0.1)
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
                new PID(1.5, 0, 0),
                new PID(1.5, 0, 0),
                new PID(2.1, 0, 0.1)
        );
    };

    public static final double drivebaseRadius = Math.hypot(driveConfig.trackWidthMeters / 2.0, driveConfig.trackLengthMeters / 2.0);
    public static final double joystickMaxAngularSpeedRadPerSec = 5.53;

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
                new PIDF(
                        0.05, 0.0,
                        // FL + FR + BL + BR
                        Util.average(0.21524, 0.16554, 0.083665, 0.061984),
                        Util.average(0.11224, 0.11693, 0.12106, 0.12449),
                        Util.average(0.0038991, 0.0018671, 0.004602, 0.0059919)
                ),
                new PID(8.0, 0.0),
                Mk4iGearRatios.L2,
                Mk4iGearRatios.TURN
        );
        case ALPHABOT -> new ModuleConfig(
                new PIDF(
                        0.05, 0.0,
                        // FL + FR + BL + BR
                        Util.average(0.024319, 0.094701 /* , [erroneous], [erroneous] */),
                        Util.average(0.13551, 0.13733, 0.13543, 0.14087),
                        Util.average(0.0065694, 0.0054738, /* [erroneous], */ 0.0091241)
                ),
                new PID(5.0, 0.0),
                Mk4iGearRatios.L2,
                Mk4iGearRatios.TURN
        );
        case SIMBOT -> new ModuleConfig(
                new PIDF(0.1, 0.0, 0.0, 0.13),
                new PID(10.0, 0.0),
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
                new ModuleIOTalonFXSparkMaxCANcoder(10, 12, 13, 0.189),
                new ModuleIOTalonFXSparkMaxCANcoder(7, 8, 9, 1.891),
                new ModuleIOTalonFXSparkMaxCANcoder(1, 2, 3, -0.009),
                new ModuleIOTalonFXSparkMaxCANcoder(5, 6, 4, 1.525),
        };
        case ALPHABOT -> new ModuleIO[]{
                // FL, FR, BL, BR
                new ModuleIOSparkMaxCANcoder(2, 3, 1, 2.454),
                new ModuleIOSparkMaxCANcoder(12, 13, 11, -0.735),
                new ModuleIOSparkMaxCANcoder(4, 5, 6, 2.623),
                new ModuleIOSparkMaxCANcoder(9, 10, 8, -1.302),
        };
        case SIMBOT -> new ModuleIO[]{
                new frc.robot.subsystems.drive.ModuleIOSim(),
                new frc.robot.subsystems.drive.ModuleIOSim(),
                new frc.robot.subsystems.drive.ModuleIOSim(),
                new ModuleIOSim()
        };
    };

    public static final frc.robot.subsystems.drive.GyroIO gyroIO = Constants.isReplay
            ? new frc.robot.subsystems.drive.GyroIO()
            : switch (Constants.identity) {
        case COMPBOT -> new frc.robot.subsystems.drive.GyroIOPigeon2(11);
        case ALPHABOT -> new GyroIOPigeon2(7);
        case SIMBOT -> new GyroIO();
    };

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
            PID choreoFeedbackXY,
            PID choreoFeedbackTheta,
            PID pointTowardsController
    ) {
    }

    public record ModuleConfig(
            PIDF driveGains,
            PID turnFeedback,
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
