package frc.robot.subsystems.superstructure;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.OperatorDashboard.LocalReefSide;
import frc.robot.OperatorDashboard.ReefZoneSide;
import frc.robot.Util;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;
import java.util.stream.Stream;

import static frc.robot.Util.shouldFlip;
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

    private static final double distanceCenterOfReefToBranchMeters = Units.inchesToMeters(6.5);

    private static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private static final Transform2d initialAlignOffset = new Transform2d(0.75, 0, new Rotation2d());
    private static final Transform2d finalAlignOffset = new Transform2d(driveConfig.bumperLengthMeters() / 2.0, 0, new Rotation2d());

    // Very rough
    public static final double initialElevatorRaiseToleranceMeters = 1.5;
    public static final double initialAlignToleranceMeters = 0.7;
    public static final double initialAlignToleranceRad = Units.degreesToRadians(20);
    public static final double initialAlignToleranceRadPerSecond = Units.degreesToRadians(20);
    // very little tolerance
    public static final double finalAlignToleranceMeters = 0.05;
    public static final double finalAlignToleranceRad = Units.degreesToRadians(2);
    public static final double finalAlignToleranceMetersPerSecond = 0.05;
    public static final double finalAlignToleranceRadPerSecond = Units.degreesToRadians(2);

    public static final double interpolateSeconds = 0.4;

    /**
     * Returns the closest reef side
     * TODO: Make this less weird
     */
    public static ReefZoneSide closestSide(Pose2d currentPose) {
        Stream<ReefZoneSide> stream = Stream.of(ReefZoneSide.values());
        Pose2d nearestPose = currentPose.nearest(stream.map(AutoAlignLocations::getAprilTagPoseAdjusted).toList());
        stream = Stream.of(ReefZoneSide.values());
        return stream.filter((value) -> (getAprilTagPoseAdjusted(value).equals(nearestPose))).findFirst().orElse(ReefZoneSide.LeftFront);
    }

    public static boolean switchSide(Pose2d currentPose, ChassisSpeeds robotRelativeSpeeds) {
        if (robotRelativeSpeeds.vxMetersPerSecond == 0 && robotRelativeSpeeds.vyMetersPerSecond == 0) {
            return false;
        }
        ReefZoneSide closestSide = closestSide(currentPose);
        Pose2d leftTagPose = getAprilTagPoseAdjusted(ReefZoneSide.getSideFromID(closestSide.aprilTagOffset - 1));
        Pose2d rightTagPose = getAprilTagPoseAdjusted(ReefZoneSide.getSideFromID(closestSide.aprilTagOffset + 1));
        Rotation2d angleToLeftTag = leftTagPose.relativeTo(currentPose).getTranslation().getAngle()
                .minus(
                        rightTagPose.relativeTo(currentPose).getTranslation().getAngle()
                );
        Logger.recordOutput("Superstructure/angleToRightTag", Util.positiveModulus(angleToLeftTag.getDegrees(), 360));
        Rotation2d angleToAdjustedPosition = new Rotation2d(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond)
                .minus(
                        rightTagPose.relativeTo(currentPose).getTranslation().getAngle()
                );
        Logger.recordOutput("Superstructure/angleToAdjustedPosition", Util.positiveModulus(angleToAdjustedPosition.getDegrees(), 360));
        return Util.positiveModulus(angleToAdjustedPosition.getRadians(), 2 * Math.PI) > Util.positiveModulus(angleToLeftTag.getRadians(), 2 * Math.PI);
    }

    public static ReefZoneSide closestSideAdjusted(Pose2d currentPose, ChassisSpeeds robotRelativeSpeeds) {
        Logger.recordOutput("Superstructure/interpolatedPosition", currentPose.exp(robotRelativeSpeeds.toTwist2d(interpolateSeconds)));
        if (switchSide(currentPose, robotRelativeSpeeds)) {
            return closestSide(currentPose.exp(robotRelativeSpeeds.toTwist2d(interpolateSeconds)));
        } else {
            return closestSide(currentPose);
        }
    }

    /**
     * Checks whether moving to the side will intersect with the reef, and refuses to do so if it does.
     * A bit rudimentary and imperfect, but definitely plenty good
     */
    public static boolean alignable(ReefZoneSide reefZoneSide, Pose2d currentPose) {
        return currentPose.relativeTo(getAprilTagPoseAdjusted(reefZoneSide)).getX() > 0;
    }

    public static Pose2d getFinalAlignPose(double elevatorPercentage, ReefZoneSide reefZoneSide, LocalReefSide localReefSide) {
        Pose2d finalAlign = getAprilTagPoseAdjusted(reefZoneSide).plus(localReefSideAdjustment(localReefSide));
        if (elevatorPercentage >= 0.9) return finalAlign;
        // If we aren't high enough, interpolate the pose from the initial align pose to the final based on elevator percentage
        Pose2d initialAlign = getInitialAlignPose(reefZoneSide, localReefSide);
        // Fully at final when 100% raised, fully at initial when 0% raised
        // .interpolate will handle values >1 or <0
        return initialAlign.interpolate(finalAlign, elevatorPercentage);
    }

    public static Pose2d getInitialAlignPose(ReefZoneSide reefZoneSide, LocalReefSide localReefSide) {
        // TODO: Maybe adjust this so you get closer to your pose?
        return getAprilTagPoseAdjusted(reefZoneSide).plus(localReefSideAdjustment(localReefSide)).plus(initialAlignOffset);
    }

    private static Transform2d localReefSideAdjustment(LocalReefSide localReefSide) {
        return new Transform2d(0, switch (localReefSide) {
            case Left -> -distanceCenterOfReefToBranchMeters;
            case Right -> distanceCenterOfReefToBranchMeters;
        }, new Rotation2d());
    }

    // See notes above, also accounts for robot size
    private static Pose2d getAprilTagPoseAdjusted(ReefZoneSide reefZoneSide) {
        return getAprilTagPoseSide(reefZoneSide.aprilTagOffset).plus(finalAlignOffset);
    }

    private static Pose2d getAprilTagPoseSide(int side) {
        if (!shouldFlip()) {
            // blue
            return getAprilTagPose(22 - ((side + 3) % 6));
        } else {
            // red
            return getAprilTagPose(side + 6);
        }
    }

    @SuppressWarnings("OptionalGetWithoutIsPresent") // better for our code to crash than to fail silently
    private static Pose2d getAprilTagPose(int id) {
        return aprilTagLayout.getTagPose(id).get().toPose2d();
    }

    public static final double stationAlignToleranceXYMeters = 0.15;
    public static final double stationAlignToleranceOmegaRad = Units.degreesToRadians(15);

    private static final double stationX = 1.53;
    private static final double stationTheta = 2.2;
    private static final Pose2d processorSideStation = new Pose2d(
            stationX,
            0.7,
            Rotation2d.fromRadians(-stationTheta)
    );
    private static final Pose2d bargeSideStation = new Pose2d(
            stationX,
            7.35,
            Rotation2d.fromRadians(stationTheta)
    );

    @RequiredArgsConstructor
    public enum Station {
        BargeSide(() -> Util.flipIfNeeded(bargeSideStation)),
        ProcessorSide(() -> Util.flipIfNeeded(processorSideStation));

        public final Supplier<Pose2d> alignPoseSupplier;
    }
}
