package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

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
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private static final double primaryAlignDistanceMeters = 1;
    private static final double secondaryAlignDistanceMeters = 0.1;

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
    public static boolean alignable(int side, Pose2d currentPose) {
        return currentPose.relativeTo(getAprilTagPoseAdjusted(side)).getX() > 0;
    }

    public static Pose2d getFinalAlignPose(int side, boolean leftSide) {
        return getAprilTagPoseAdjusted(side).plus(new Transform2d(
                0,
                leftSide ? Units.inchesToMeters(-6.5) : Units.inchesToMeters(6.5),
                new Rotation2d()
        ));
    }

    public static Pose2d getSecondaryAlignPose(int side, boolean leftSide) {
        return getAprilTagPoseAdjusted(side).plus(new Transform2d(
                secondaryAlignDistanceMeters,
                // right relative to the pose since the pose faces away from the reef
                leftSide ? Units.inchesToMeters(-6.5) : Units.inchesToMeters(6.5),
                new Rotation2d()
        ));
    }

    public static Pose2d getPrimaryAlignPose(int side) {
        // TODO: Maybe adjust this so you get closer to your pose?
        return getAprilTagPoseAdjusted(side).plus(new Transform2d(primaryAlignDistanceMeters, 0, new Rotation2d()));
    }

    // See notes above, also accounts for robot size
    public static Pose2d getAprilTagPoseAdjusted(int side) {
        if (!shouldFlip()) {
            // blue
            return getAprilTagPose(22 - ((side + 3) % 6)).plus(new Transform2d(Units.inchesToMeters(17.25), 0, new Rotation2d()));
        } else {
            // red
            return getAprilTagPose(side + 6).plus(new Transform2d(Units.inchesToMeters(17.25), 0, new Rotation2d()));
        }
    }

    public static Pose2d getAprilTagPose(int id) {
        return aprilTagLayout.getTagPose(id).get().toPose2d();
    }
}
