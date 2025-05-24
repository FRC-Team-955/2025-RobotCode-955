package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.PIDF;
import frc.robot.util.swerve.ModuleLimits;

import java.util.function.BiConsumer;

public class DriveConstants {
    public static final double assistDirectionToleranceRad = Units.degreesToRadians(50);
    public static final double assistMaximumDistanceMeters = Units.feetToMeters(5);

    public static final MoveToConfig moveToConfig = new MoveToConfig(
            PIDF.ofPD(4.5, 0.05),
            PIDF.ofPD(4.5, 0.05),
            PIDF.ofPD(1, 0).profiled(3.8, 5),
            PIDF.ofPD(2, 0).profiled(3, 3),
            0.02,
            0.1,
            Units.degreesToRadians(2),
            Units.degreesToRadians(10)
    );

    public static void calculateMoveToProfiledLinearConstraints(TrapezoidProfile.Constraints overallConstraints, Rotation2d directionOfTravel, BiConsumer<TrapezoidProfile.Constraints, TrapezoidProfile.Constraints> applyXYConstraints) {
        Translation2d maxVelocities = new Pose2d(new Translation2d(), directionOfTravel)
                .transformBy(new Transform2d(overallConstraints.maxVelocity, 0, new Rotation2d()))
                .getTranslation();
        Translation2d maxAccelerations = new Pose2d(new Translation2d(), directionOfTravel)
                .transformBy(new Transform2d(overallConstraints.maxAcceleration, 0, new Rotation2d()))
                .getTranslation();
        applyXYConstraints.accept(
                // X
                new TrapezoidProfile.Constraints(Math.abs(maxVelocities.getX()), Math.abs(maxAccelerations.getX())),
                // Y
                new TrapezoidProfile.Constraints(Math.abs(maxVelocities.getY()), Math.abs(maxAccelerations.getY()))
        );
    }

    public static final boolean useSetpointGenerator = true;
    public static final boolean disableDriving = false;
    public static final boolean disableGyro = false;
    public static final boolean useHighFrequencyOdometry = true;

    public static final double odometryPositionDeltaDiscardMeters = Units.inchesToMeters(8);

    // Slow to 30% speed during driver control
    public static final double constraintScalarWhenElevatorAtMaxHeightDriver = 0.3;

    public static final DriveConfig driveConfig = switch (Constants.identity) {
        case COMPBOT -> new DriveConfig(
                Units.inchesToMeters(1.935948620917915),
                Units.inchesToMeters(22.75),
                Units.inchesToMeters(22.75),
                Units.inchesToMeters(35),
                Units.inchesToMeters(35),
                PIDF.ofPD(3.5, 0),
                PIDF.ofPD(3, 0),
                new ModuleLimits(
                        1.5,
                        15,
                        20
                )
        );
        case SIMBOT -> new DriveConfig(
                Units.inchesToMeters(2),
                Units.inchesToMeters(22.75),
                Units.inchesToMeters(22.75),
                Units.inchesToMeters(35),
                Units.inchesToMeters(35),
                PIDF.ofPD(3.5, 0),
                PIDF.ofPD(3, 0),
                new ModuleLimits(
                        3.83,
                        25,
                        20
                )
        );
    };

    public static final ModuleLimits moveToModuleLimits = new ModuleLimits(
            driveConfig.moduleLimits().maxDriveVelocityMetersPerSec(),
            driveConfig.moduleLimits().maxDriveAccelerationMetersPerSecSquared() * 0.75,
            driveConfig.moduleLimits().maxTurnVelocityRadPerSec()
    );

    /**
     * FL, FR, BL, BR
     */
    public static final Translation2d[] moduleTranslations = new Translation2d[]{
            new Translation2d(driveConfig.trackWidthMeters / 2.0, driveConfig.trackLengthMeters / 2.0),
            new Translation2d(driveConfig.trackWidthMeters / 2.0, -driveConfig.trackLengthMeters / 2.0),
            new Translation2d(-driveConfig.trackWidthMeters / 2.0, driveConfig.trackLengthMeters / 2.0),
            new Translation2d(-driveConfig.trackWidthMeters / 2.0, -driveConfig.trackLengthMeters / 2.0)
    };

    public static final double drivebaseRadiusMeters = Math.hypot(driveConfig.trackWidthMeters / 2.0, driveConfig.trackLengthMeters / 2.0);

    /** Maximum angular velocity of the whole drivetrain if all drive motors/wheels are going at full speed. */
    public static final double maxAngularVelocityRadPerSec = driveConfig.moduleLimits().maxDriveVelocityMetersPerSec() / drivebaseRadiusMeters;

    public static final double joystickMaxAngularSpeedRadPerSec = Math.min(Units.degreesToRadians(315), maxAngularVelocityRadPerSec);
    public static final double joystickDriveDeadband = 0.1;

    public static final ModuleConfig moduleConfig = switch (Constants.identity) {
        case COMPBOT -> new ModuleConfig(
                PIDF.ofPDSVA(
                        0.0, 0.0,
                        0.19, 0.125, 0.005
                ),
                PIDF.ofPD(5, 0.04),
                Mk4iGearRatios.L2,
                Mk4iGearRatios.TURN,
                true,
                false,
                false,
                120,
                60
        );
        case SIMBOT -> new ModuleConfig(
                PIDF.ofPDSV(0.05, 0.0, 0.04075, 0.14117),
                PIDF.ofPD(10.0, 0.07),
                Mk4iGearRatios.L2,
                Mk4iGearRatios.TURN,
                true,
                false,
                false,
                120,
                60
        );
    };

    public static ModuleIO[] createModuleIO() {
        if (Constants.isReplay) {
            return new ModuleIO[]{new ModuleIO(), new ModuleIO(), new ModuleIO(), new ModuleIO()};
        }
        return switch (Constants.identity) {
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
            case SIMBOT -> new ModuleIO[]{
                    new ModuleIOSim(0),
                    new ModuleIOSim(1),
                    new ModuleIOSim(2),
                    new ModuleIOSim(3)
            };
        };
    }

    public static GyroIO createGyroIO() {
        if (Constants.isReplay) {
            return new GyroIO();
        }
        return switch (Constants.identity) {
            case COMPBOT -> new GyroIOPigeon2(9);
            case SIMBOT -> new GyroIOSim();
        };
    }

    public record MoveToConfig(
            PIDF pureLinear,
            PIDF pureAngular,
            PIDF.Profiled profiledLinear,
            PIDF.Profiled profiledAngular,
            double linearPositionToleranceMeters,
            double linearVelocityToleranceMetersPerSec,
            double angularPositionToleranceRad,
            double angularVelocityToleranceRadPerSec
    ) {
    }

    public record DriveConfig(
            double wheelRadiusMeters,
            double trackWidthMeters, // Measured from the center of the swerve wheels
            double trackLengthMeters,
            double bumperWidthMeters,
            double bumperLengthMeters,
            PIDF choreoFeedbackXY,
            PIDF choreoFeedbackOmega,
            ModuleLimits moduleLimits // See ModuleLimits for docs on each value
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
