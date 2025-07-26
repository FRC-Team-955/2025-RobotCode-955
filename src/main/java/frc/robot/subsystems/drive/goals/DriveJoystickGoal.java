package frc.robot.subsystems.drive.goals;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Controller;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Supplier;

public class DriveJoystickGoal {
    private static final RobotState robotState = RobotState.get();
    private static final Controller controller = Controller.get();

    @Setter
    private static Supplier<Optional<Pose2d>> assistPoseSupplier = Optional::empty;

    public static Pair<ChassisSpeeds, Drive.Goal> get() {
        var optionalAssistPose = assistPoseSupplier.get();
        if (optionalAssistPose.isPresent()) {
            // Mark assist pose as present
            Logger.recordOutput("Drive/Assist/Present", true);
            Pose2d assistPose = optionalAssistPose.get();

            if (controller.shouldAssist(robotState.getPose(), assistPose)) {
                return new Pair<>(
                        getAssisted(assistPose),
                        Drive.Goal.DRIVE_JOYSTICK_ASSISTED
                );
            }
        }

        Logger.recordOutput("Drive/Assist/Present", false);
        return new Pair<>(
                controller.getDriveSetpointRobotRelative(robotState.getRotation()),
                Drive.Goal.DRIVE_JOYSTICK
        );
    }

    private static ChassisSpeeds getAssisted(Pose2d assistPose) {
        var currentPose = robotState.getPose();

        double assistX = 0;
        double assistY = 0;
        double assistOmega = 0;
        // TODO: need to reset the PIDs when assist starts
        // TODO: log setpoint
//        double assistX = moveToLinearX.calculate(
//                currentPose.getX(),
//                assistPose.getX()
//        ) + moveToLinearX.getSetpoint().velocity;
//        assistX *= linearMagnitude; // Limit to the driver's overall linear speed
//
//        double assistY = moveToLinearY.calculate(
//                currentPose.getY(),
//                assistPose.getY()
//        ) + moveToLinearY.getSetpoint().velocity;
//        assistY *= linearMagnitude; // Limit to the driver's overall linear speed
//
//        double assistOmega = moveToAngular.calculate(
//                currentPose.getRotation().getRadians(),
//                assistPose.getRotation().getRadians()
//        ) + moveToAngular.getSetpoint().velocity;
//        // If we are driving fast and not rotating, need fast rotation assist, so limit to driver's overall linear speed
//        // Otherwise, limit to driver omega speed
//        assistOmega *= Math.max(omegaMagnitude, linearMagnitude);

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                        assistX,
                        assistY,
                        assistOmega,
                        currentPose.getRotation() // Move to is absolute, don't flip
                )
                .times(0.75)
                .plus(controller.getDriveSetpointRobotRelative(currentPose.getRotation()).times(0.25));
    }
}
