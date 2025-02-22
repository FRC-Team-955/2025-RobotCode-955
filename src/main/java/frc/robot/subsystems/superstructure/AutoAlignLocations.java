package frc.robot.subsystems.superstructure;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.OperatorDashboard.LocalReefSide;
import frc.robot.OperatorDashboard.ReefZoneSide;

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

    // Very rough - 50 cm and 30 degrees
    public static final double initialAlignToleranceMeters = 0.5;
    public static final double initialAlignToleranceRad = 0.5;
    // very little tolerance - 2 cm, 2 deg
    public static final double finalAlignToleranceMeters = 0.04;
    public static final double finalAlignToleranceRad = 0.03;
    public static final double finalAlignToleranceMetersPerSecond = 0.04;
    public static final double finalAlignToleranceRadPerSecond = 0.03;

    /**
     * Checks whether moving to the side will intersect with the reef, and refuses to do so if it does.
     * A bit rudimentary and imperfect, but definitely plenty good
     */
    public static boolean alignable(ReefZoneSide reefZoneSide, Pose2d currentPose) {
        return currentPose.relativeTo(getAprilTagPoseAdjusted(reefZoneSide)).getX() > 0;
    }

    public static Pose2d getFinalAlignPose(ReefZoneSide reefZoneSide, LocalReefSide localReefSide) {
        return getAprilTagPoseAdjusted(reefZoneSide).plus(localReefSideAdjustment(localReefSide));
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
}
