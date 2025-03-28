package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Util;
import frc.robot.subsystems.apriltagvision.AprilTagVisionConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.AllianceBasedPose2d;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.Comparator;
import java.util.EnumMap;
import java.util.Map;

import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

public class AutoAlignLocations {
    /**
     * Notes:
     * Sides will be labeled from the left to right, starting with the farthest left the driver can see at 0.
     * Thus, the side with the apriltag facing the driver is side 1
     * <p>
     * Auto Align will use the default fast PID to move to a position primaryAlignDistanceMeters out from the april tag
     * Finally, it will move forward the remaining amount and place the gamepiece
     */

    @SuppressWarnings("OptionalGetWithoutIsPresent") // better for our code to crash than to fail silently
    private static Pose2d getAprilTagPose(int id) {
        return AprilTagVisionConstants.aprilTagLayout.getTagPose(id).get().toPose2d();
    }

    private static final Transform2d bumperOffset = new Transform2d(driveConfig.bumperLengthMeters() / 2.0, 0, new Rotation2d());

    public static class ReefAlign {
        private static final double distanceCenterOfReefToBranchMeters = Units.inchesToMeters(6.5);
        private static final double distanceCenterOfReefToElevatorClearanceMeters = distanceCenterOfReefToBranchMeters + Units.inchesToMeters(8);

        private static final Transform2d initialAlignStartOffset = new Transform2d(1, 0, new Rotation2d());
        private static final Transform2d initialAlignEndOffset = new Transform2d(0.4, 0, new Rotation2d());
        private static final double initialAlignDistForStartMeters = 1.0;

        private static final double finalAlignAngularDiffForInitialRad = Units.degreesToRadians(30);

        // Distance at which to start raising the elevator
        public static final double elevatorRaiseDistanceMeters = 1.5;
        // Distance at which elevator cannot be raised
        public static final double elevatorStowDistanceMeters = initialAlignEndOffset.getX() / 2.0;
        public static final double elevatorRaiseAngularToleranceRad = Units.degreesToRadians(45);

        public static final double alignLinearToleranceMeters = 0.04;
        public static final double alignAngularToleranceRad = Units.degreesToRadians(4);
        public static final double alignLinearToleranceMetersPerSecond = 0.08;
        public static final double alignAngularToleranceRadPerSecond = Units.degreesToRadians(8);

        public static boolean canRaiseElevator(Pose2d currentPose, ReefZoneSide reefZoneSide, LocalReefSide localReefSide) {
            Pose2d finalAlign = getFinalAlignPose(reefZoneSide, localReefSide);
            Transform2d relative = new Transform2d(finalAlign, currentPose);

            double distance = Math.abs(relative.getTranslation().getNorm());
            boolean distanceMet = distance < elevatorRaiseDistanceMeters && distance > elevatorStowDistanceMeters;

            boolean rotationMet = Math.abs(MathUtil.angleModulus(relative.getRotation().getRadians())) < elevatorRaiseAngularToleranceRad;

            Logger.recordOutput("Superstructure/ReefAlign/ElevatorDistanceMet", distanceMet);
            Logger.recordOutput("Superstructure/ReefAlign/ElevatorRotationMet", rotationMet);

            return distanceMet && rotationMet;
        }

        public static boolean atFinalAlign(Pose2d currentPose, ChassisSpeeds measuredChassisSpeeds, ReefZoneSide reefZoneSide, LocalReefSide localReefSide) {
            Pose2d finalAlign = getFinalAlignPose(reefZoneSide, localReefSide);
            boolean positionMet = Util.isAtPoseWithTolerance(currentPose, finalAlign, alignLinearToleranceMeters, alignAngularToleranceRad);
            boolean velocityMet = Util.isWithinVelocityTolerance(measuredChassisSpeeds, alignLinearToleranceMetersPerSecond, alignAngularToleranceRadPerSecond);
            Logger.recordOutput("Superstructure/ReefAlign/PositionMet", positionMet);
            Logger.recordOutput("Superstructure/ReefAlign/VelocityMet", velocityMet);
            return positionMet && velocityMet;
        }

        /**
         * Checks whether moving to the side will intersect with the reef, and refuses to do so if it does.
         * A bit rudimentary and imperfect, but definitely plenty good
         */
        public static boolean isAlignable(Pose2d currentPose, ReefZoneSide reefZoneSide) {
            return currentPose.relativeTo(reefZoneSide.getAdjustedAprilTagPose()).getX() > -0.15;
        }

        public static Pose2d getFinalAlignPose(ReefZoneSide reefZoneSide, LocalReefSide localReefSide) {
            return reefZoneSide.getAdjustedAprilTagPose().plus(localReefSide.finalAdjust);
        }

        public static Pose2d getAlignPose(Pose2d currentPose, double elevatorPercentage, ReefZoneSide reefZoneSide, LocalReefSide localReefSide) {
            Pose2d base = reefZoneSide.getAdjustedAprilTagPose();
            Pose2d finalAlign = getFinalAlignPose(reefZoneSide, localReefSide);


            // If elevator isn't close enough, start by calculating the initial align
            Pose2d initialBase = base.plus(localReefSide.initialAdjust);
            Pose2d initialStart = initialBase.plus(initialAlignStartOffset);
            Pose2d initialEnd = initialBase.plus(initialAlignEndOffset);
            // Interpolate to initialBase based on y distance (left/right distance)
            double initialDistY = Math.abs(new Transform2d(initialEnd, currentPose).getY());
            // No clamping needed, Pose2d.interpolate will handle it
            // Note: initialInterp is inverted (0 = end, 1 = start)
            double initialInterp = initialDistY / initialAlignDistForStartMeters;
            Pose2d initial = initialEnd.interpolate(initialStart, initialInterp);

            // Now interpolate from initial to final end based on elevator percentage
            // Fully at final when 100% raised, fully at initial when 0% raised
            if (elevatorPercentage >= 0.9) {
                elevatorPercentage = 1.0;
            } else {
                elevatorPercentage = MathUtil.clamp(elevatorPercentage, 0.0, 1.0);
            }

            // Also consider rotational difference when aligning - we don't want to fully align if we aren't pointing in the right direction
            double finalAngularDiff = Math.abs(MathUtil.angleModulus(new Transform2d(finalAlign, currentPose).getRotation().getRadians()));
            if (finalAngularDiff < alignAngularToleranceRad / 2.0) {
                finalAngularDiff = 0.0;
            }
            // Clamp needed since we're multiplying
            double angularDiffInterp = MathUtil.clamp(1.0 - (finalAngularDiff / finalAlignAngularDiffForInitialRad), 0.0, 1.0);

            return initial.interpolate(finalAlign, elevatorPercentage * angularDiffInterp);
        }

        public static AllianceBasedPose2d getAdjustedReefAprilTagPose(int aprilTagOffset) {
            return new AllianceBasedPose2d(
                    // blue
                    getAprilTagPose(22 - ((aprilTagOffset + 3) % 6)).plus(bumperOffset),

                    // red
                    getAprilTagPose(aprilTagOffset + 6).plus(bumperOffset)
            );
        }

        private static final double velocityLookaheadSeconds = 0.4;
        private static final Transform2d reefSideAngleOffset = new Transform2d(0.25, 0, new Rotation2d());

        private static final EnumMap<ReefZoneSide, Pose2d> reefZoneSideToAdjustedPose = Util.createEnumMap(ReefZoneSide.class, ReefZoneSide.values(), ReefZoneSide::getAdjustedAprilTagPose);

        private static ReefZoneSide closestReefSideToPose(Pose2d currentPose) {
            return reefZoneSideToAdjustedPose.entrySet()
                    .stream()
                    .min(Comparator.comparing(
                            entry -> currentPose.getTranslation().getDistance(entry.getValue().getTranslation())
                    ))
                    .map(Map.Entry::getKey)
                    .orElse(ReefZoneSide.LeftFront);
        }

        public static ReefZoneSide determineClosestReefSide(Pose2d currentPose, ChassisSpeeds joystickSetpointFieldRelative) {
            ReefZoneSide closestSide = closestReefSideToPose(currentPose);
            boolean lookahead = false;

            if (joystickSetpointFieldRelative.vxMetersPerSecond != 0 || joystickSetpointFieldRelative.vyMetersPerSecond != 0) {
                // Get poses of sides left and right of closest, with an offset
                Pose2d leftTagPose = ReefZoneSide.fromOrdinal(closestSide.ordinal() - 1).getAdjustedAprilTagPose()
                        .plus(reefSideAngleOffset);
//        Logger.recordOutput("Superstructure/ClosestReefSide/LeftTagPose", leftTagPose);
                Pose2d rightTagPose = ReefZoneSide.fromOrdinal(closestSide.ordinal() + 1).getAdjustedAprilTagPose()
                        .plus(reefSideAngleOffset);
//        Logger.recordOutput("Superstructure/ClosestReefSide/RightTagPose", rightTagPose);

                // Get angles to left and right
                Rotation2d robotToLeft = leftTagPose.relativeTo(currentPose).getTranslation().getAngle();
//            Logger.recordOutput("Superstructure/ClosestReefSide/RobotToLeft", robotToLeft);
                Rotation2d robotToRight = rightTagPose.relativeTo(currentPose).getTranslation().getAngle();
//            Logger.recordOutput("Superstructure/ClosestReefSide/RobotToRight", robotToRight);

                // Get the joystick angle relative to the closest tag
                Rotation2d joystickAngle = new Rotation2d(joystickSetpointFieldRelative.vxMetersPerSecond, joystickSetpointFieldRelative.vyMetersPerSecond);
//            Logger.recordOutput("Superstructure/ClosestReefSide/JoystickAngle", joystickAngle);

                // Lookahead if the angle of the joystick setpoint angle is more CCW (positive) than the left pose or more CW (negative) than the right pose
                // We have to make all rotations relative to the center tag so that CCW and CW are actually positive and negative and the inequalities work out
                Pose2d centerTagPose = closestSide.getAdjustedAprilTagPose();
                Rotation2d relativeToCenterTag = centerTagPose.getRotation().unaryMinus().plus(Rotation2d.k180deg);
                lookahead = joystickAngle.rotateBy(relativeToCenterTag).getRadians() > robotToLeft.rotateBy(relativeToCenterTag).getRadians() ||
                        joystickAngle.rotateBy(relativeToCenterTag).getRadians() < robotToRight.rotateBy(relativeToCenterTag).getRadians();
            }
            Pose2d currentPoseWithLookahead = currentPose.exp(ChassisSpeeds.fromFieldRelativeSpeeds(joystickSetpointFieldRelative, currentPose.getRotation()).toTwist2d(velocityLookaheadSeconds));
//        Logger.recordOutput("Superstructure/ClosestReefSide/Lookahead", currentPoseWithLookahead);
            if (lookahead) {
                return closestReefSideToPose(currentPoseWithLookahead);
            } else {
                return closestSide;
            }
        }

        // Distance at which to raise the elevator when descoring
        public static final double descoreElevatorRaiseDistanceMeters = 1.5;
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
            LeftFront(getAdjustedReefAprilTagPose(0), Elevator.Goal.DESCORE_L2),
            MiddleFront(getAdjustedReefAprilTagPose(1), Elevator.Goal.DESCORE_L3),
            RightFront(getAdjustedReefAprilTagPose(2), Elevator.Goal.DESCORE_L2),
            RightBack(getAdjustedReefAprilTagPose(3), Elevator.Goal.DESCORE_L3),
            MiddleBack(getAdjustedReefAprilTagPose(4), Elevator.Goal.DESCORE_L2),
            LeftBack(getAdjustedReefAprilTagPose(5), Elevator.Goal.DESCORE_L3);

            private final AllianceBasedPose2d adjustedAprilTagPoses;
            public final Elevator.Goal algaeDescoringElevatorGoal;

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
            Left(
                    new Transform2d(0, -distanceCenterOfReefToElevatorClearanceMeters, new Rotation2d()),
                    new Transform2d(0, -distanceCenterOfReefToBranchMeters, new Rotation2d())
            ),
            Right(
                    new Transform2d(0, distanceCenterOfReefToElevatorClearanceMeters, new Rotation2d()),
                    new Transform2d(0, distanceCenterOfReefToBranchMeters, new Rotation2d())
            ),
            Middle(new Transform2d(), new Transform2d()),
            ;

            public final Transform2d initialAdjust;
            public final Transform2d finalAdjust;
        }
    }

    public static class StationAlign {
        public static final double alignLinearToleranceMeters = 0.05;
        public static final double alignAngularToleranceRad = Units.degreesToRadians(10);

        private static final Transform2d alignOffsetBargeSide = new Transform2d(0, 0.6, Rotation2d.k180deg);
        private static final Transform2d alignOffsetProcessorSide = new Transform2d(
                alignOffsetBargeSide.getX(),
                -alignOffsetBargeSide.getY(),
                alignOffsetBargeSide.getRotation()
        );
        private static final Transform2d alignOffsetProcessorSideFriendly = new Transform2d(
                alignOffsetProcessorSide.getX(),
                alignOffsetProcessorSide.getY() + 1.15,
                alignOffsetProcessorSide.getRotation()
        );

        @RequiredArgsConstructor
        public enum Station {
            BargeSide(StationAlign.getAlignPose(1, alignOffsetBargeSide)),
            ProcessorSide(StationAlign.getAlignPose(0, alignOffsetProcessorSide)),
            ProcessorSideFriendly(StationAlign.getAlignPose(0, alignOffsetProcessorSideFriendly));

            private final AllianceBasedPose2d alignPose;

            public Pose2d getAlignPose() {
                return alignPose.get();
            }
        }

        private static AllianceBasedPose2d getAlignPose(int aprilTagOffset, Transform2d alignOffset) {
            return new AllianceBasedPose2d(
                    // blue
                    // 12 = processor side, 13 = barge side
                    getAprilTagPose(12 + (aprilTagOffset % 2)).plus(bumperOffset).plus(alignOffset),

                    // red
                    // 2 = processor side, 1 = barge side
                    getAprilTagPose(1 + ((1 - aprilTagOffset) % 2)).plus(bumperOffset).plus(alignOffset)
            );
        }

        public static boolean atAlignPose(Pose2d currentPose, Station station) {
            Pose2d align = station.getAlignPose();
            boolean positionMet = Util.isAtPoseWithTolerance(currentPose, align, alignLinearToleranceMeters, alignAngularToleranceRad);
            Logger.recordOutput("Superstructure/StationAlign/PositionMet", positionMet);
            return positionMet;
        }
    }
}
