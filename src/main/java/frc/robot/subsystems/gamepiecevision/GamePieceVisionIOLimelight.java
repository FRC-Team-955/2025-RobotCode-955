package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

public class GamePieceVisionIOLimelight extends GamePieceVisionIO {

    private static final double CONNECTION_TIMEOUT_MS = 250.0;
    private static final double CAMERA_HEIGHT_METERS = 0.45;
    private static final double CORAL_HEIGHT_METERS = 0.05;
    private static final double CAMERA_PITCH_RADIANS = Math.toRadians(20.0);

    private final DoubleSubscriber latencySubscriber;
    private final BooleanSubscriber targetValidSubscriber;
    private final DoubleSubscriber txSubscriber;
    private final DoubleSubscriber tySubscriber;
    private final DoublePublisher ledModePublisher;

    private boolean ledsOn;

    public GamePieceVisionIOLimelight(String tableName) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);

        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        targetValidSubscriber = table.getBooleanTopic("tv").subscribe(false);
        txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
        tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
        ledModePublisher = table.getDoubleTopic("ledMode").publish();

        setLEDs(false);
    }

    @Override
    public void updateInputs(GamePieceVisionIOInputs inputs) {
        double timeSinceUpdateMs =
                (RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000.0;

        inputs.connected = timeSinceUpdateMs < CONNECTION_TIMEOUT_MS;
        inputs.ledsOn = ledsOn;
        inputs.visible = targetValidSubscriber.get();
        inputs.coralYaw = new Rotation2d();
        inputs.coralPitch = new Rotation2d();
        inputs.coralPos = new Pose2d();

        if (!inputs.visible) {
            return;
        }

        double yawRadians = Math.toRadians(txSubscriber.get());
        double pitchRadians = Math.toRadians(tySubscriber.get());

        inputs.coralYaw = Rotation2d.fromRadians(yawRadians);
        inputs.coralPitch = Rotation2d.fromRadians(pitchRadians);

        double distanceMeters =
                (CORAL_HEIGHT_METERS - CAMERA_HEIGHT_METERS) /
                        Math.tan(CAMERA_PITCH_RADIANS + pitchRadians);

        Translation2d coralTranslation =
                new Translation2d(distanceMeters, new Rotation2d(yawRadians));

        inputs.coralPos = new Pose2d(coralTranslation, new Rotation2d());
    }

    @Override
    public void setLEDs(boolean on) {
        if (ledsOn == on) {
            return;
        }

        ledsOn = on;
        ledModePublisher.accept(on ? 3.0 : 1.0);
        NetworkTableInstance.getDefault().flush();
    }
}
