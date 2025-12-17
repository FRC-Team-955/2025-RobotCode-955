package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystem.Periodic;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.util.commands.CommandsExt;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static frc.robot.subsystems.gamepiecevision.GamePieceVisionConstants.camToRobot;
import static frc.robot.subsystems.gamepiecevision.GamePieceVisionConstants.createIO;

public class GamePieceVision implements Periodic {
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final GamePieceVisionIO io = createIO();
    private final GamePieceVisionIOInputsAutoLogged inputs = new GamePieceVisionIOInputsAutoLogged();

    private final Alert disconnectedAlert = new Alert("Game piece vision is disconnected.", Alert.AlertType.kError);

    private final Debouncer visibleDebouncer = new Debouncer(0.03);

    private static GamePieceVision instance;

    public static GamePieceVision get() {
        if (instance == null) {
            synchronized (GamePieceVision.class) {
                instance = new GamePieceVision();
            }
        }
        return instance;
    }

    private GamePieceVision() {}

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        computeCoralPose();
        Logger.processInputs("Inputs/GamePieceVision", inputs);
        disconnectedAlert.set(!inputs.connected);
    }

    public boolean visibleNotDebounced() {
        return inputs.connected && inputs.visible;
    }

    @AutoLogOutput(key = "GamePieceVision/VisibleDebounced")
    public boolean visibleDebounced() {
        return inputs.connected && visibleDebouncer.calculate(inputs.visible);
    }

    public Command waitForGamePiece() {
        return CommandsExt.startEndWaitUntil(
                () -> io.setLEDs(true),
                () -> io.setLEDs(false),
                this::visibleDebounced
        );
    }

    public Pose2d getCoralPos() {
        return inputs.coralPos;
    }

    public boolean getVisibility() {
        return inputs.visible;
    }

    private void computeCoralPose() {
        if (!inputs.connected || inputs.targetObservations.length == 0) {
            inputs.visible = false;
            return;
        }

        Pose2d robotPose = RobotState.get().getPose();

        Translation2d closest = null;
        double closestDistance = Double.MAX_VALUE;

        for (var obs : inputs.targetObservations) {
            double distance =
                    (Units.inchesToMeters(4.5) - camToRobot.getTranslation().getZ())
                            / Math.tan(GamePieceVisionConstants.cameraPitch + obs.pitchRad());

            if (!Double.isFinite(distance) || distance <= 0) continue;

            Translation2d robotRelative =
                    new Translation2d(
                            distance * Math.sin(obs.yawRad()),
                            distance * Math.cos(obs.yawRad())
                    );
            Translation2d fieldRelative =
                    robotPose.getTranslation().plus(robotRelative.rotateBy(robotPose.getRotation()));

            double distToRobot = fieldRelative.getDistance(robotPose.getTranslation());
            if (distToRobot < closestDistance) {
                closestDistance = distToRobot;
                closest = fieldRelative;
            }
        }

        if (closest != null) {
            inputs.coralPos = new Pose2d(closest, new Rotation2d());
            inputs.visible = true;
        } else {
            inputs.visible = false;
        }
    }
}
