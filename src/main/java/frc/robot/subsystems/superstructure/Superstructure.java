package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Util;
import frc.lib.subsystem.CommandBasedSubsystem;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.createIO;

public class Superstructure extends CommandBasedSubsystem {
    private final RobotState robotState = RobotState.get();
    private final GamePieceVision gamePieceVision = GamePieceVision.get();
    private final Drive drive = Drive.get();

    private final SuperstructureIO io = createIO();
    private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE,

        AUTO_INTAKE_SEARCHING,
        AUTO_INTAKE_SEARCHING_FOR_STALE,
        AUTO_INTAKE_INTAKING,
        ;
    }

    @Getter
    private Goal goal = Goal.IDLE;

    public Command setGoal(Goal superstructureGoal) {
        return runOnce(() -> {
            goal = superstructureGoal;
        });
    }

    private static Superstructure instance;

    public static Superstructure get() {
        if (instance == null)
            synchronized (Superstructure.class) {
                instance = new Superstructure();
            }

        return instance;
    }

    private Superstructure() {
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superstructure", inputs);
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Goal", goal);
    }

    public Command cancel() {
        return setGoal(Goal.IDLE).ignoringDisable(true);
    }

    public Command autoIntakeCoral() {
        // TODO: searching mode (turn left and right and slowly move forward) when no coral
        // TODO: don't end until intaked coral - CANrange
        // TODO: different goals for different states
        Timer searchingTimer = new Timer();
        final Rotation2d searchRange = Rotation2d.fromDegrees(60);
        final double lookPeriodSecs = 5.0;
        var state = new Object() {
            Rotation2d originalHeading = null;
            Pose2d currentSetpoint = null;
            Pose2d finalSetpoint = null;
        };

        Supplier<Goal> calculateSetpoint = () -> {
            Pose2d pose = robotState.getPose();
            if (state.originalHeading == null) {
                state.originalHeading = pose.getRotation();
            }

            Translation2d closestCoral = null;
            for (var coral : gamePieceVision.getFreshCoral()) {
                Translation2d coralTranslation = coral.toPose2d().getTranslation();
                if (closestCoral == null || closestCoral.getDistance(pose.getTranslation()) > coralTranslation.getDistance(pose.getTranslation())) {
                    closestCoral = coralTranslation;
                }
            }

            if (closestCoral != null) {
                // found fresh coral
                searchingTimer.stop();
                Pose2d closestCoralFacingRobot = new Pose2d(
                        closestCoral,
                        closestCoral.minus(pose.getTranslation()).getAngle()
                );
                Pose2d setpoint = closestCoralFacingRobot.transformBy(new Transform2d(-0.75, 0, new Rotation2d()));

                state.finalSetpoint = setpoint;
                state.currentSetpoint = setpoint;

                return Goal.AUTO_INTAKE_INTAKING;
            }

            for (var coral : gamePieceVision.getStaleCoral()) {
                Translation2d coralTranslation = coral.toPose2d().getTranslation();
                if (closestCoral == null || closestCoral.getDistance(pose.getTranslation()) > coralTranslation.getDistance(pose.getTranslation())) {
                    closestCoral = coralTranslation;
                }
            }

            if (closestCoral != null) {
                // Stale coral
                if (!searchingTimer.isRunning()) {
                    searchingTimer.restart();
                }
                Rotation2d facingTowardsCoral = closestCoral.minus(pose.getTranslation()).getAngle();

                Rotation2d lookToRight = facingTowardsCoral.plus(searchRange);
                Rotation2d lookToLeft = facingTowardsCoral.minus(searchRange);

                Logger.recordOutput("Superstructure/AutoIntakeCoral/SearchingTimer", searchingTimer.get());
                double lookInterpolation = 0.5 + 0.5 * Math.cos(2.0 * Math.PI * (searchingTimer.get() % lookPeriodSecs) / lookPeriodSecs);
                Logger.recordOutput("Superstructure/AutoIntakeCoral/LookInterpolation", lookInterpolation);

                state.currentSetpoint = new Pose2d(
                        new Pose2d(
                                pose.getTranslation(),
                                facingTowardsCoral
                        ).transformBy(new Transform2d(0.1, 0, new Rotation2d())).getTranslation(),
                        lookToLeft.interpolate(lookToRight, lookInterpolation)
                );
                state.finalSetpoint = null;

                return Goal.AUTO_INTAKE_SEARCHING_FOR_STALE;
            }

            // No coral
            if (!searchingTimer.isRunning()) {
                searchingTimer.restart();
            }

            Rotation2d lookToRight = state.originalHeading.plus(searchRange);
            Rotation2d lookToLeft = state.originalHeading.minus(searchRange);
            Logger.recordOutput("Superstructure/AutoIntakeCoral/SearchingTimer", searchingTimer.get());

            double lookInterpolation = 0.5 + 0.5 * Math.cos(2.0 * Math.PI * (searchingTimer.get() % lookPeriodSecs) / lookPeriodSecs);
            Logger.recordOutput("Superstructure/AutoIntakeCoral/LookInterpolation", lookInterpolation);
            Pose2d forwardPose = new Pose2d(
                    pose.getTranslation(),
                    state.originalHeading
            ).transformBy(new Transform2d(0.05, 0, new Rotation2d()));

            state.currentSetpoint = new Pose2d(
                    forwardPose.getTranslation(),
                    lookToLeft.interpolate(lookToRight, lookInterpolation)
            );
            state.finalSetpoint = null;

            return Goal.AUTO_INTAKE_SEARCHING;
        };

        return Commands.parallel(
                Commands.runOnce(() -> state.originalHeading = null),
                Commands.run(() -> goal = calculateSetpoint.get()),
                drive.moveTo(() -> state.currentSetpoint, false)
        ).until(() -> {
            if (state.finalSetpoint == null) {
                return false;
            }
            return Util.isAtPoseWithTolerance(robotState.getPose(), state.finalSetpoint, DriveConstants.moveToConfig.linearPositionToleranceMeters(), DriveConstants.moveToConfig.angularPositionToleranceRad());
        });
    }
}
