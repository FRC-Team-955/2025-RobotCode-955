package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.lib.AllianceBasedPose2d;
import frc.lib.Util;
import frc.lib.network.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Comparator;

public class ReefAlign {
    private static final double distanceCenterOfReefToBranchMeters = Units.inchesToMeters(6.5);

    private static final Transform2d initialAlignStartOffset = new Transform2d(1.5, 0, new Rotation2d());
    private static final Transform2d initialAlignEndOffset = new Transform2d(0.5, 0, new Rotation2d());
    private static final double initialAlignDistYForStartMeters = 1.5;
    private static final double initialAlignDistYOffset = 0.5;
    private static final double initialAlignDistXForFullAngle = 0.5;

    private static final double finalAlignAngularDiffForInitialRad = Units.degreesToRadians(30);
    private static final double finalAlignElevatorPercentageMultiplier = 1.25;

    public static final double alignLinearToleranceMeters = 0.04;
    public static final double alignAngularToleranceRad = Units.degreesToRadians(4);
    public static final double alignLinearToleranceMetersPerSecond = 0.02;
    public static final double alignAngularToleranceRadPerSecond = Units.degreesToRadians(8);

    public static boolean atFinalAlign(Pose2d currentPose, ChassisSpeeds measuredChassisSpeeds, ReefZoneSide reefZoneSide, LocalReefSide localReefSide) {
        Pose2d finalAlign = getFinalAlignPose(reefZoneSide, localReefSide);
        boolean positionMet = Util.isAtPoseWithTolerance(currentPose, finalAlign, alignLinearToleranceMeters, alignAngularToleranceRad);
        boolean velocityMet = Util.isWithinVelocityTolerance(measuredChassisSpeeds, alignLinearToleranceMetersPerSecond, alignAngularToleranceRadPerSecond);
        Logger.recordOutput("Superstructure/ReefAlign/PositionMet", positionMet);
        Logger.recordOutput("Superstructure/ReefAlign/VelocityMet", velocityMet);
        return positionMet && velocityMet;
    }

    public static boolean isAlignable(Pose2d currentPose, ReefZoneSide reefZoneSide) {
        return currentPose.relativeTo(reefZoneSide.getAdjustedAprilTagPose()).getX() > -0.15;
    }

    public static Pose2d getFinalAlignPose(ReefZoneSide reefZoneSide, LocalReefSide localReefSide) {
        return reefZoneSide.getAdjustedAprilTagPose().plus(localReefSide.adjust);
    }

    public static Pose2d getAlignPose(Pose2d currentPose, double elevatorPercentage, ReefZoneSide reefZoneSide, LocalReefSide localReefSide) {
        Pose2d finalAlign = getFinalAlignPose(reefZoneSide, localReefSide);
        Pose2d initialStart = finalAlign.plus(initialAlignStartOffset);
        Pose2d initialEnd = finalAlign.plus(initialAlignEndOffset);

        double initialDistY = Math.abs(new Transform2d(initialEnd, currentPose). getY()) - initialAlignDistYOffset;

        double initialInterp = 1.0 - (initialDistY / initialAlignDistYForStartMeters);
        return initialStart.interpolate(initialEnd, initialInterp);
    }

    public static final int[] reefTagIds = {
            // Blue
            17,
            18,
            19,
            20,
            21,
            22,

            // Red
            6,
            7,
            8,
            9,
            10,
            11
    };

    private static AllianceBasedPose2d getAdjustedReefAprilTagPose(int aprilTagOffset) {
        return new AllianceBasedPose2d(
                // Blue
                AlignHelpers.getAprilTagPose(22 - ((aprilTagOffset + 3) % 6)).plus(AlignHelpers.bumperOffset),

                // Red
                AlignHelpers.getAprilTagPose(aprilTagOffset + 6).plus(AlignHelpers.bumperOffset)
        );
    }

    private static final LoggedTunableNumber velocityLookaheadSeconds = new LoggedTunableNumber("ReefAlign/VelocityLookaheadSeconds", 0.4);
    private static final Transform2d reefSideAngleOffset = new Transform2d(0.25, 0, new Rotation2d());

    private static ReefZoneSide closestReefSideToPose(Pose2d currentPose) {
        return Arrays.stream(ReefZoneSide.values())
                .min(Comparator.comparing(
                        side -> currentPose.getTranslation().getDistance(side.getAdjustedAprilTagPose().getTranslation())
                ))
                .orElse(ReefZoneSide.LeftFront);
    }


    public static ReefZoneSide determineClosestReefSide(Pose2d currentPose, ChassisSpeeds joystickSetpointFieldRelative) {
        ReefZoneSide closestSide = closestReefSideToPose(currentPose);
        boolean lookahead = false;

        if (joystickSetpointFieldRelative.vxMetersPerSecond != 0 || joystickSetpointFieldRelative.vyMetersPerSecond != 0) {
            Pose2d leftTagPose = ReefZoneSide.fromOrdinal(closestSide.ordinal() - 1).getAdjustedAprilTagPose()
                    .plus(reefSideAngleOffset);
            Pose2d rightTagPose = ReefZoneSide.fromOrdinal(closestSide.ordinal() + 1).getAdjustedAprilTagPose()
                    .plus(reefSideAngleOffset);

            Rotation2d robotToLeft = leftTagPose.relativeTo(currentPose).getTranslation().getAngle();
            Rotation2d robotToRight = rightTagPose.relativeTo(currentPose).getTranslation().getAngle();

            Rotation2d joystickAngle = new Rotation2d(joystickSetpointFieldRelative.vxMetersPerSecond, joystickSetpointFieldRelative.vyMetersPerSecond);

            Pose2d centerTagPose = closestSide.getAdjustedAprilTagPose();
            Rotation2d relativeToCenterTag = centerTagPose.getRotation().unaryMinus().plus(Rotation2d.k180deg);
            lookahead = joystickAngle.rotateBy(relativeToCenterTag).getRadians() > robotToLeft.rotateBy(relativeToCenterTag).getRadians() ||
                    joystickAngle.rotateBy(relativeToCenterTag).getRadians() < robotToRight.rotateBy(relativeToCenterTag).getRadians();
        }
        Pose2d currentPoseWithLookahead = currentPose.exp(ChassisSpeeds.fromFieldRelativeSpeeds(joystickSetpointFieldRelative, currentPose.getRotation()).toTwist2d(velocityLookaheadSeconds.get()));
        if (lookahead) {
            return closestReefSideToPose(currentPoseWithLookahead);
        } else {
            return closestSide;
        }
    }

    public static final double descoreElevatorRaiseDistanceMeters = 1.0;
    private static final double descoreLinearToleranceMeters = 0.1;
    private static final double descoreAngularToleranceRad = Units.degreesToRadians(10);

    public static Pose2d getDescoreAlignPose(ReefZoneSide reefZoneSide) {
        return ReefAlign.getFinalAlignPose(reefZoneSide, LocalReefSide.Middle);
    }

    public static boolean descoreCanRaiseElevator(Pose2d currentPose, ReefZoneSide reefZoneSide) {
        Pose2d alignPose = getDescoreAlignPose(reefZoneSide);
        return Util.isAtPoseWithTolerance(
                currentPose,
                alignPose,
                ReefAlign.descoreElevatorRaiseDistanceMeters,
                ReefAlign.descoreAngularToleranceRad
        );
    }

    public static boolean descoreIsAligned(Pose2d currentPose, ReefZoneSide reefZoneSide) {
        Pose2d alignPose = getDescoreAlignPose(reefZoneSide);
        return Util.isAtPoseWithTolerance(
                currentPose,
                alignPose,
                ReefAlign.descoreLinearToleranceMeters,
                ReefAlign.descoreAngularToleranceRad
        );
    }



        @RequiredArgsConstructor
        public enum ReefZoneSide {
            LeftFront(getAdjustedReefAprilTagPose(0)),
            MiddleFront(getAdjustedReefAprilTagPose(1)),
            RightFront(getAdjustedReefAprilTagPose(2)),
            RightBack(getAdjustedReefAprilTagPose(3)),
            MiddleBack(getAdjustedReefAprilTagPose(4)),
            LeftBack(getAdjustedReefAprilTagPose(5));

            private final AllianceBasedPose2d adjustedAprilTagPoses;

            public static ReefZoneSide fromOrdinal(int ordinal) {
                var values = ReefZoneSide.values();
                return values[Util.positiveModulus(ordinal, values.length)];
            }

            public Pose2d getAdjustedAprilTagPose() {
                return adjustedAprilTagPoses.get();
            }
        }

        @RequiredArgsConstructor
        public enum LocalReefSide {
            Left(new Transform2d(0, -distanceCenterOfReefToBranchMeters, new Rotation2d())),
            Right(new Transform2d(0, distanceCenterOfReefToBranchMeters, new Rotation2d())),
            Middle(new Transform2d()),
            ;

            public final Transform2d adjust;
        }

}
