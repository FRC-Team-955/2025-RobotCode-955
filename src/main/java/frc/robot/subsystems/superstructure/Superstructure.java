package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Util;
import frc.lib.commands.CommandsExt;
import frc.lib.subsystem.CommandBasedSubsystem;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;
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

        AUTO_INTAKE,
        ;
    }

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
        Supplier<Pose2d> setpointSupplier = () -> {
            Pose2d pose = robotState.getPose();

            Translation2d closestCoral = null;
            double xOffset = -0.75;
            for (var coral : gamePieceVision.getFreshCoral()) {
                Translation2d coralTranslation = coral.toPose2d().getTranslation();
                if (closestCoral == null || closestCoral.getDistance(pose.getTranslation()) > coralTranslation.getDistance(pose.getTranslation())) {
                    closestCoral = coralTranslation;
                }
            }
            if (closestCoral == null) {
                xOffset = -1.5;
                for (var coral : gamePieceVision.getStaleCoral()) {
                    Translation2d coralTranslation = coral.toPose2d().getTranslation();
                    if (closestCoral == null || closestCoral.getDistance(pose.getTranslation()) > coralTranslation.getDistance(pose.getTranslation())) {
                        closestCoral = coralTranslation;
                    }
                }
            }
            if (closestCoral != null) {
                Pose2d closestCoralFacingRobot = new Pose2d(
                        closestCoral,
                        closestCoral.minus(pose.getTranslation()).getAngle()
                );
                return closestCoralFacingRobot.transformBy(new Transform2d(xOffset, 0, new Rotation2d()));
            }
            return pose;
        };

        return CommandsExt.eagerSequence(
                setGoal(Goal.AUTO_INTAKE),
                drive.moveTo(setpointSupplier, false)
                        .until(() -> Util.isAtPoseWithTolerance(robotState.getPose(), setpointSupplier.get(), DriveConstants.moveToConfig.linearPositionToleranceMeters(), DriveConstants.moveToConfig.angularPositionToleranceRad()))
        );
    }
}
