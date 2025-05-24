package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Util;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.drive.DriveConstants.*;

public class JoystickDrive {
    // Intermediates - used in assist calculations
    private Rotation2d linearDirection = new Rotation2d();
    private double linearMagnitude = 0.0;

    // Results - used for drive/assist calculations
    @Getter
    private ChassisSpeeds setpointFieldRelative = new ChassisSpeeds();

    private static JoystickDrive instance;

    public static JoystickDrive get() {
        if (instance == null)
            synchronized (JoystickDrive.class) {
                instance = new JoystickDrive();
            }

        return instance;
    }

    private JoystickDrive() {
    }

    public void update(double x, double y, double omega) {
        Logger.recordOutput("JoystickDrive/Suppliers/X", x);
        Logger.recordOutput("JoystickDrive/Suppliers/Y", y);
        Logger.recordOutput("JoystickDrive/Suppliers/Omega", omega);

        linearMagnitude = MathUtil.clamp(MathUtil.applyDeadband(Math.hypot(x, y), joystickDriveDeadband), -1, 1);
        linearMagnitude = linearMagnitude * linearMagnitude;

        double omegaMagnitude = MathUtil.applyDeadband(omega, joystickDriveDeadband);
        omegaMagnitude = Math.copySign(omegaMagnitude * omegaMagnitude, omegaMagnitude);

        // Scale linear magnitude by omega - when going full omega, want half linear
        linearMagnitude *= MathUtil.clamp(1.0 - Math.abs(omegaMagnitude / 2.0), 0.5, 1.0);

        // If x and y are both 0, Rotation2d will not be happy
        // Intermediates - used in assist calculations
        Translation2d linearVelocity;
        if (x != 0 || y != 0) {
            linearDirection = new Rotation2d(x, y);
            linearVelocity = new Pose2d(new Translation2d(), linearDirection)
                    .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                    .getTranslation();
        } else {
            // Linear magnitude should be 0 anyways
            linearDirection = new Rotation2d();
            linearVelocity = new Translation2d();
        }

        Logger.recordOutput("JoystickDrive/LinearMagnitude", linearMagnitude);
        Logger.recordOutput("JoystickDrive/LinearDirection", linearDirection);
        Logger.recordOutput("JoystickDrive/LinearVelocity", linearVelocity);
        Logger.recordOutput("JoystickDrive/OmegaMagnitude", omegaMagnitude);

//        if (Util.shouldFlip()) {
        linearVelocity = linearVelocity.rotateBy(Rotation2d.k180deg);
//        }
        setpointFieldRelative = new ChassisSpeeds(
                linearVelocity.getX() * driveConfig.moduleLimits().maxDriveVelocityMetersPerSec(),
                linearVelocity.getY() * driveConfig.moduleLimits().maxDriveVelocityMetersPerSec(),
                omegaMagnitude * joystickMaxAngularSpeedRadPerSec
        );
    }

    public boolean shouldAssist(Pose2d currentPose, Pose2d assistPose) {
        Logger.recordOutput("JoystickDrive/Assist/Pose", assistPose);

        // Get the translation between robot and assist
        Translation2d robotToAssist = assistPose.getTranslation().minus(currentPose.getTranslation());
        // Calculate direction from robot to assist
        Rotation2d robotToAssistDirection = robotToAssist.getAngle();
        Logger.recordOutput("JoystickDrive/Assist/RobotToAssistDirection", robotToAssistDirection);

        // Flip joystick direction to match robot to assist direction
        // Joystick direction is relative to alliance wall and needs to be flipped on red alliance to match origin
        Rotation2d joystickLinearDirectionFlipped = Util.flipIfNeeded(linearDirection);
        Logger.recordOutput("JoystickDrive/Assist/FlippedJoystickLinearDirection", joystickLinearDirectionFlipped);

        // Get difference between joystick direction and assist direction
        Rotation2d directionDiff = robotToAssistDirection.minus(joystickLinearDirectionFlipped);
        Logger.recordOutput("JoystickDrive/Assist/DirectionDifference", directionDiff);

        // Get distance to assist pose
        double distanceToAssist = currentPose.getTranslation().getDistance(assistPose.getTranslation());
        Logger.recordOutput("JoystickDrive/Assist/DistanceToAssist", distanceToAssist);

        // If we are:
        if (
            // - moving linearly in some way (if we are only rotating, don't assist)
                linearMagnitude != 0.0 &&
                        // - going towards the assist pose based on threshold
                        Math.abs(directionDiff.getRadians()) < assistDirectionToleranceRad &&
                        // - close enough to assist pose
                        distanceToAssist < assistMaximumDistanceMeters
        ) {
            // then use automatic control.
            Logger.recordOutput("JoystickDrive/Assist/Running", true);
            return true;
        } else {
            Logger.recordOutput("JoystickDrive/Assist/Running", false);
            return false;
        }
    }

    public ChassisSpeeds getSetpointRobotRelative(Rotation2d robotAngle) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(setpointFieldRelative, robotAngle);
    }
}
