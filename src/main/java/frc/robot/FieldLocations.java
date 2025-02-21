package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import lombok.RequiredArgsConstructor;

import java.util.function.Supplier;

import static frc.robot.Util.shouldFlip;

public class FieldLocations {
    /* Auto Align */

    /**
     * Notes:
     * Sides will be labeled from the left to right, starting with the farthest left the driver can see at 0.
     * Thus, the side with the apriltag facing the driver is side 1
     * <p>
     * Auto Align will use the default fast PID to move to a position primaryAlignDistanceMeters out from the april tag
     * Then it will raise the elevator while using a slower PID to move to a position secondaryAlignDistanceMeters from the reef node
     * Finally, it will move forward the remaining amount and place the gamepiece
     */
    @RequiredArgsConstructor
    public enum ReefZoneSide {
        LEFT_FRONT(() -> getAprilTagPoseSide(0)),
        MIDDLE_FRONT(() -> getAprilTagPoseSide(1)),
        RIGHT_FRONT(() -> getAprilTagPoseSide(2)),
        RIGHT_BACK(() -> getAprilTagPoseSide(3)),
        MIDDLE_BACK(() -> getAprilTagPoseSide(4)),
        LEFT_BACK(() -> getAprilTagPoseSide(5));

        public final Supplier<Pose2d> tagPose;
    }

    @RequiredArgsConstructor
    public enum LocalReefSide {
        LEFT(() -> new Transform2d(0, Units.inchesToMeters(-6.5), new Rotation2d())),
        RIGHT(() -> new Transform2d(0, Units.inchesToMeters(6.5), new Rotation2d()));

        public final Supplier<Transform2d> adjustPose;
    }

    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private static final double primaryAlignDistanceMeters = 1;
    private static final Transform2d primaryAlignPosition = new Transform2d(1, 0, new Rotation2d());
    private static final double secondaryAlignDistanceMeters = 0.1;
    private static final Transform2d secondaryAlignPosition = new Transform2d(0.1, 0, new Rotation2d());

    // Very rough - 30 cm and 30 degrees
    public static final double primaryAlignToleranceMeters = 0.3;
    public static final double primaryAlignToleranceRad = 0.5;
    // 4 cm and 5 degrees - angle is more important here
    public static final double secondaryAlignToleranceMeters = 0.04;
    public static final double secondaryAlignToleranceRad = 0.08;
    // very little tolerance - 2 cm, 2 deg
    // TODO: Verify we're actually good enough for this accuracy
    public static final double finalAlignToleranceMeters = 0.04;
    public static final double finalAlignToleranceRad = 0.03;
    public static final double finalAlignToleranceMetersPerSecond = 0.04;
    public static final double finalAlignToleranceRadPerSecond = 0.03;

    public static Rotation2d getAngle(Pose2d initialPose, Pose2d finalPose) {
        return finalPose.getRotation().minus(initialPose.getRotation());
    }

    public static double getDistance(Pose2d initialTarget, Pose2d finalTarget) {
        return finalTarget.relativeTo(initialTarget).getTranslation().getNorm();
    }

    /**
     * Checks whether moving to the side will intersect with the reef, and refuses to do so if it does.
     * A bit rudimentary and imperfect, but definitely plenty good
     */
    public static boolean alignable(ReefZoneSide reefZoneSide, Pose2d currentPose) {
        return currentPose.relativeTo(getAprilTagPoseAdjusted(reefZoneSide)).getX() > 0;
    }

    public static Pose2d getFinalAlignPose(ReefZoneSide reefZoneSide, LocalReefSide localReefSide) {
        return getAprilTagPoseAdjusted(reefZoneSide).plus(localReefSide.adjustPose.get());
    }

    public static Pose2d getSecondaryAlignPose(ReefZoneSide reefZoneSide, LocalReefSide localReefSide) {
        return getAprilTagPoseAdjusted(reefZoneSide).plus(localReefSide.adjustPose.get()).plus(secondaryAlignPosition);
    }

    public static Pose2d getPrimaryAlignPose(ReefZoneSide reefZoneSide, LocalReefSide localReefSide) {
        // TODO: Maybe adjust this so you get closer to your pose?
        return getAprilTagPoseAdjusted(reefZoneSide).plus(localReefSide.adjustPose.get()).plus(primaryAlignPosition);
    }

    // See notes above, also accounts for robot size
    public static Pose2d getAprilTagPoseAdjusted(ReefZoneSide reefZoneSide) {
        return reefZoneSide.tagPose.get().plus(new Transform2d(Units.inchesToMeters(17.25), 0, new Rotation2d()));
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

    public static Pose2d getAprilTagPose(int id) {
        return aprilTagLayout.getTagPose(id).get().toPose2d();
    }
}
