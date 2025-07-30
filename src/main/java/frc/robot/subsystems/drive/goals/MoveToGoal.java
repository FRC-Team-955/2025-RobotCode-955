package frc.robot.subsystems.drive.goals;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Controller;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.subsystems.drive.DriveTuning.*;

public class MoveToGoal {
    private static final RobotState robotState = RobotState.get();
    private static final Controller controller = Controller.get();
    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final Drive drive = Drive.get();

    private static BooleanSupplier mergeJoystickDrive = () -> false;
    private static Supplier<Pose2d> poseSupplier = robotState::getPose;

    private static final PIDController moveToPureLinearX = moveToConfig.pureLinear().toPID(
            moveToConfig.linearPositionToleranceMeters(),
            moveToConfig.linearVelocityToleranceMetersPerSec()
    );
    private static final PIDController moveToPureLinearY = moveToConfig.pureLinear().toPID(
            moveToConfig.linearPositionToleranceMeters(),
            moveToConfig.linearVelocityToleranceMetersPerSec()
    );
    private static final PIDController moveToPureAngular = moveToConfig.pureAngular().toPIDWrapRadians(
            moveToConfig.angularPositionToleranceRad(),
            moveToConfig.angularVelocityToleranceRadPerSec()
    );

    private static final ProfiledPIDController moveToProfiledLinearX = moveToConfig.profiledLinear().toPID(
            moveToConfig.linearPositionToleranceMeters(),
            moveToConfig.linearVelocityToleranceMetersPerSec()
    );
    private static final ProfiledPIDController moveToProfiledLinearY = moveToConfig.profiledLinear().toPID(
            moveToConfig.linearPositionToleranceMeters(),
            moveToConfig.linearVelocityToleranceMetersPerSec()
    );
    private static final ProfiledPIDController moveToProfiledAngular = moveToConfig.profiledAngular().toPIDWrapRadians(
            moveToConfig.angularPositionToleranceRad(),
            moveToConfig.angularVelocityToleranceRadPerSec()
    );

    public static void initialize(Supplier<Pose2d> newPoseSupplier, BooleanSupplier newMergeJoystickDrive) {
        poseSupplier = newPoseSupplier;
        mergeJoystickDrive = newMergeJoystickDrive;

        if (operatorDashboard.profiledMoveTo.get()) {
            Pose2d currentPose = robotState.getPose();
            ChassisSpeeds currentVelocities = drive.getMeasuredChassisSpeedsFieldRelative();
            moveToProfiledLinearX.reset(
                    currentPose.getX(),
                    currentVelocities.vxMetersPerSecond
            );
            moveToProfiledLinearY.reset(
                    currentPose.getY(),
                    currentVelocities.vyMetersPerSecond
            );
            moveToProfiledAngular.reset(
                    MathUtil.angleModulus(currentPose.getRotation().getRadians()),
                    currentVelocities.omegaRadiansPerSecond
            );
        } else {
            moveToPureLinearX.reset();
            moveToPureLinearY.reset();
            moveToPureAngular.reset();
        }
    }

    public static Pair<ChassisSpeeds, Drive.Goal> get() {
        moveToPureLinearTunable.ifChanged(gains -> {
            gains.applyPID(moveToPureLinearX);
            gains.applyPID(moveToPureLinearY);
        });
        moveToPureAngularTunable.ifChanged(gains -> gains.applyPID(moveToPureAngular));

        moveToProfiledLinearTunable.ifChanged(
                gains -> {
                    gains.applyPID(moveToProfiledLinearX);
                    gains.applyPID(moveToProfiledLinearY);
                },
                // Constraints will be calculated and applied in move to
                constraints -> {
                }
        );
        moveToProfiledAngularTunable.ifChanged(
                gains -> gains.applyPID(moveToProfiledAngular),
                moveToProfiledAngular::setConstraints
        );

        //////////////////////////////////////////////////////////////////////

        Pose2d currentPose = robotState.getPose();

        Pose2d goalPose = poseSupplier.get();
        Logger.recordOutput("Drive/MoveTo/Goal", goalPose);

        double linearXVelocityMetersPerSec;
        boolean linearXAtSetpoint;
        double linearYVelocityMetersPerSec;
        boolean linearYAtSetpoint;
        double angularVelocityRadPerSec;
        boolean angularAtSetpoint;
        if (operatorDashboard.profiledMoveTo.get()) {
            ChassisSpeeds currentVelocities = drive.getMeasuredChassisSpeedsFieldRelative();

            // Reset if it just changed
            if (operatorDashboard.profiledMoveTo.hasChanged()) {
                moveToProfiledLinearX.reset(
                        currentPose.getX(),
                        currentVelocities.vxMetersPerSecond
                );
                moveToProfiledLinearY.reset(
                        currentPose.getY(),
                        currentVelocities.vyMetersPerSecond
                );
                moveToProfiledAngular.reset(
                        MathUtil.angleModulus(currentPose.getRotation().getRadians()),
                        currentVelocities.omegaRadiansPerSecond
                );
            }

            // Update profile constraints
            Translation2d currentToGoal = goalPose.getTranslation().minus(currentPose.getTranslation());
            if (currentToGoal.getX() != 0 || currentToGoal.getY() != 0) {
                Rotation2d directionOfTravel = currentToGoal.getAngle();
                calculateMoveToProfiledLinearConstraints(moveToProfiledLinearTunable.getConstraints(), directionOfTravel, (x, y) -> {
                    moveToProfiledLinearX.setConstraints(x);
                    moveToProfiledLinearY.setConstraints(y);
                });
            }

            linearXVelocityMetersPerSec = moveToProfiledLinearX.calculate(
                    currentPose.getX(),
                    goalPose.getX()
            ) + moveToProfiledLinearX.getSetpoint().velocity;
            linearXAtSetpoint = moveToProfiledLinearX.atSetpoint();

            linearYVelocityMetersPerSec = moveToProfiledLinearY.calculate(
                    currentPose.getY(),
                    goalPose.getY()
            ) + moveToProfiledLinearY.getSetpoint().velocity;
            linearYAtSetpoint = moveToProfiledLinearY.atSetpoint();

            angularVelocityRadPerSec = moveToProfiledAngular.calculate(
                    MathUtil.angleModulus(currentPose.getRotation().getRadians()),
                    MathUtil.angleModulus(goalPose.getRotation().getRadians())
            ) + moveToProfiledAngular.getSetpoint().velocity;
            angularAtSetpoint = moveToProfiledAngular.atSetpoint();

            Logger.recordOutput("Drive/MoveTo/Setpoint", new Pose2d(
                    moveToProfiledLinearX.getSetpoint().position,
                    moveToProfiledLinearY.getSetpoint().position,
                    new Rotation2d(moveToProfiledAngular.getSetpoint().position)
            ));
        } else {
            // Reset if it just changed
            if (operatorDashboard.profiledMoveTo.hasChanged()) {
                moveToPureLinearX.reset();
                moveToPureLinearY.reset();
                moveToPureAngular.reset();
            }

            linearXVelocityMetersPerSec = moveToPureLinearX.calculate(
                    currentPose.getX(),
                    goalPose.getX()
            );
            linearXAtSetpoint = moveToPureLinearX.atSetpoint();

            linearYVelocityMetersPerSec = moveToPureLinearY.calculate(
                    currentPose.getY(),
                    goalPose.getY()
            );
            linearYAtSetpoint = moveToPureLinearY.atSetpoint();

            angularVelocityRadPerSec = moveToPureAngular.calculate(
                    MathUtil.angleModulus(currentPose.getRotation().getRadians()),
                    MathUtil.angleModulus(goalPose.getRotation().getRadians())
            );
            angularAtSetpoint = moveToPureAngular.atSetpoint();
        }

        Logger.recordOutput("Drive/MoveTo/LinearXAtSetpoint", linearXAtSetpoint);
        if (linearXAtSetpoint) {
            linearXVelocityMetersPerSec = 0.0;
        }

        Logger.recordOutput("Drive/MoveTo/LinearYAtSetpoint", linearYAtSetpoint);
        if (linearYAtSetpoint) {
            linearYVelocityMetersPerSec = 0.0;
        }

        Logger.recordOutput("Drive/MoveTo/AngularAtSetpoint", angularAtSetpoint);
        if (angularAtSetpoint) {
            angularVelocityRadPerSec = 0.0;
        }

        // Scale linear speed by angular speed so that angular change is prioritized
        // When going max angular speed, reduce linear to 75%
        double linearScalar = MathUtil.clamp(1 - angularVelocityRadPerSec / maxAngularVelocityRadPerSec, 0.75, 1);

        ChassisSpeeds moveToSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                linearXVelocityMetersPerSec * linearScalar,
                linearYVelocityMetersPerSec * linearScalar,
                angularVelocityRadPerSec,
                currentPose.getRotation() // Move to is absolute, don't flip
        );
        if (mergeJoystickDrive.getAsBoolean()) {
            ChassisSpeeds joystickDriveSpeeds = controller.getDriveSetpointRobotRelative(robotState.getRotation());
            return new Pair<>(
                    moveToSpeeds.plus(joystickDriveSpeeds.times(0.3)),
                    Drive.Goal.MOVE_TO_DRIVE_JOYSTICK_MERGED
            );
        } else {
            return new Pair<>(
                    moveToSpeeds,
                    Drive.Goal.MOVE_TO
            );
        }
    }
}
