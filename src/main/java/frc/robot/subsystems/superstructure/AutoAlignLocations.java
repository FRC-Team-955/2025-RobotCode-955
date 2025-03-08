package frc.robot.subsystems.superstructure;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.OperatorDashboard.LocalReefSide;
import frc.robot.OperatorDashboard.ReefZoneSide;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

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

    private static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    @SuppressWarnings("OptionalGetWithoutIsPresent") // better for our code to crash than to fail silently
    private static Pose2d getAprilTagPose(int id) {
        return aprilTagLayout.getTagPose(id).get().toPose2d();
    }

    private static final Transform2d bumperOffset = new Transform2d(driveConfig.bumperLengthMeters() / 2.0, 0, new Rotation2d());

    private static final double distanceCenterOfReefToBranchMeters = Units.inchesToMeters(6.5);
    private static final double distanceCenterOfReefToElevatorClearanceMeters = distanceCenterOfReefToBranchMeters + Units.inchesToMeters(10);

    private static final Transform2d initialAlignStartOffset = new Transform2d(1, 0, new Rotation2d());
    private static final Transform2d initialAlignEndOffset = new Transform2d(0.3, 0, new Rotation2d());
    private static final double initialAlignDistYForStartMeters = 2.0;

    // Very rough
    public static final double initialAlignToleranceXMeters = 0.5;
    public static final double initialAlignToleranceYMeters = 0.2;
    public static final double initialAlignToleranceRad = Units.degreesToRadians(20);
    public static final double initialAlignToleranceRadPerSecond = Units.degreesToRadians(20);

    // very little tolerance
    public static final double finalAlignToleranceMeters = 0.05;
    public static final double finalAlignToleranceRad = Units.degreesToRadians(4);
    public static final double finalAlignToleranceMetersPerSecond = 0.03;
    public static final double finalAlignToleranceRadPerSecond = Units.degreesToRadians(2);

    /**
     * Checks whether moving to the side will intersect with the reef, and refuses to do so if it does.
     * A bit rudimentary and imperfect, but definitely plenty good
     */
    public static boolean alignable(ReefZoneSide reefZoneSide, Pose2d currentPose) {
        return currentPose.relativeTo(getReefAprilTagPoseAdjusted(reefZoneSide)).getX() > 0;
    }

    public static Pose2d getFinalAlignPose(double elevatorPercentage, ReefZoneSide reefZoneSide, LocalReefSide localReefSide) {
        Pose2d base = getReefAprilTagPoseAdjusted(reefZoneSide);
        Pose2d end = base.plus(finalLocalReefSideAdjustment(localReefSide));
        if (elevatorPercentage >= 0.9) return end;
        // If we aren't high enough, interpolate the pose from the start pose to the end pose based on elevator percentage
        Pose2d start = base.plus(initialLocalReefSideAdjustment(localReefSide)).plus(initialAlignEndOffset);
        // Fully at final when 100% raised, fully at initial when 0% raised
        // .interpolate will handle values >1 or <0
        return start.interpolate(end, elevatorPercentage);
    }

    public static Pose2d getInitialAlignPose(Pose2d currentPose, ReefZoneSide reefZoneSide, LocalReefSide localReefSide) {
        Pose2d base = getReefAprilTagPoseAdjusted(reefZoneSide).plus(initialLocalReefSideAdjustment(localReefSide));
        Pose2d start = base.plus(initialAlignStartOffset);
        Pose2d end = base.plus(initialAlignEndOffset);
        // Interpolate to end based on y distance (left/right distance)
        double distY = Math.abs(new Transform2d(end, currentPose).getY());
        double t = MathUtil.clamp(distY / initialAlignDistYForStartMeters, 0, 1);
        return end.interpolate(start, t);
    }

    private static final Transform2d adjustmentLeft = new Transform2d(0, -distanceCenterOfReefToBranchMeters, new Rotation2d());
    private static final Transform2d adjustmentRight = new Transform2d(0, distanceCenterOfReefToBranchMeters, new Rotation2d());
    private static final Transform2d adjustmentLeftRaise = new Transform2d(0, -distanceCenterOfReefToElevatorClearanceMeters, new Rotation2d());
    private static final Transform2d adjustmentRightRaise = new Transform2d(0, distanceCenterOfReefToElevatorClearanceMeters, new Rotation2d());

    private static Transform2d finalLocalReefSideAdjustment(LocalReefSide localReefSide) {
        return switch (localReefSide) {
            case Left -> adjustmentLeft;
            case Right -> adjustmentRight;
        };
    }

    private static Transform2d initialLocalReefSideAdjustment(LocalReefSide localReefSide) {
        return switch (localReefSide) {
            case Left -> adjustmentLeftRaise;
            case Right -> adjustmentRightRaise;
        };
    }

    private static Pose2d getReefAprilTagPoseAdjusted(ReefZoneSide reefZoneSide) {
        if (!shouldFlip()) {
            // blue
            return getAprilTagPose(22 - ((reefZoneSide.aprilTagOffset + 3) % 6)).plus(bumperOffset);
        } else {
            // red
            return getAprilTagPose(reefZoneSide.aprilTagOffset + 6).plus(bumperOffset);
        }
    }

    public static final double stationAlignToleranceXYMeters = 0.1;
    public static final double stationAlignToleranceOmegaRad = Units.degreesToRadians(10);

    private static final Transform2d stationAlignOffset = new Transform2d(0, 0.6, Rotation2d.k180deg);

    @RequiredArgsConstructor
    public enum Station {
        BargeSide(1),
        ProcessorSide(0);

        private final int aprilTagOffset;
    }

    private static Pose2d getStationAprilTagPoseAdjusted(Station station) {
        if (!shouldFlip()) {
            // blue
            // 12 = processor side, 13 = barge side
            return getAprilTagPose(12 + (station.aprilTagOffset % 2)).plus(bumperOffset);
        } else {
            // red
            // 2 = processor side, 1 = barge side
            return getAprilTagPose(1 + ((1 - station.aprilTagOffset) % 2)).plus(bumperOffset);
        }
    }

    public static Pose2d getStationAlignPose(Station station) {
        Logger.recordOutput("Superstructrue/align", getStationAprilTagPoseAdjusted(station));
        return getStationAprilTagPoseAdjusted(station).plus(stationAlignOffset);
    }
}
