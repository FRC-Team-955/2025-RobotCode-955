package frc.robot.subsystems.drive.goals;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.swerve.ModuleLimits;
import frc.robot.Controller;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveRequest;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.subsystems.drive.DriveTuning.*;

@RequiredArgsConstructor
public class MoveToGoal extends DriveGoal {
    private static final RobotState robotState = RobotState.get();
    private static final Controller controller = Controller.get();
    private static final Drive drive = Drive.get();

    private final Supplier<Pose2d> poseSupplier;
    private final boolean mergeJoystickDrive;

    private final PIDController moveToPureLinearX = moveToConfig.pureLinear().toPID(
            moveToConfig.linearPositionToleranceMeters(),
            moveToConfig.linearVelocityToleranceMetersPerSec()
    );
    private final PIDController moveToPureLinearY = moveToConfig.pureLinear().toPID(
            moveToConfig.linearPositionToleranceMeters(),
            moveToConfig.linearVelocityToleranceMetersPerSec()
    );
    private final PIDController moveToPureAngular = moveToConfig.pureAngular().toPIDWrapRadians(
            moveToConfig.angularPositionToleranceRad(),
            moveToConfig.angularVelocityToleranceRadPerSec()
    );

    private final ProfiledPIDController moveToProfiledLinearX = moveToConfig.profiledLinear().toPID(
            moveToConfig.linearPositionToleranceMeters(),
            moveToConfig.linearVelocityToleranceMetersPerSec()
    );
    private final ProfiledPIDController moveToProfiledLinearY = moveToConfig.profiledLinear().toPID(
            moveToConfig.linearPositionToleranceMeters(),
            moveToConfig.linearVelocityToleranceMetersPerSec()
    );
    private final ProfiledPIDController moveToProfiledAngular = moveToConfig.profiledAngular().toPIDWrapRadians(
            moveToConfig.angularPositionToleranceRad(),
            moveToConfig.angularVelocityToleranceRadPerSec()
    );

    private boolean profiledNeedsReset = true;

    @Override
    public DriveRequest getRequest() {
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
        if (useProfiledMoveTo) {
            // Reset if we just started
            if (profiledNeedsReset) {
                profiledNeedsReset = false;

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

        Logger.recordOutput("Drive/MoveTo/MergeJoystickDrive", mergeJoystickDrive);
        if (mergeJoystickDrive) {
            ChassisSpeeds joystickDriveSpeeds = controller.getDriveSetpointRobotRelative(robotState.getRotation());
            return DriveRequest.chassisSpeedsOptimized(moveToSpeeds.plus(joystickDriveSpeeds.times(0.3)));
        } else {
            return DriveRequest.chassisSpeedsOptimized(moveToSpeeds);
        }
    }

    @Override
    public ModuleLimits getModuleLimits() {
        return moveToModuleLimits;
    }
}
