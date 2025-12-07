package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

import java.util.LinkedList;
import java.util.List;

public class GamePieceVisionIOLimelight extends GamePieceVisionIO {
    private final DoubleSubscriber latencySubscriber;
    private final DoubleArraySubscriber rawDetectionsSubscriber;

    public GamePieceVisionIOLimelight(String name) {
        var table = NetworkTableInstance.getDefault().getTable(name);

        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        rawDetectionsSubscriber = table.getDoubleArrayTopic("rawdetections").subscribe(new double[0]);
    }

    @Override
    public void updateInputs(GamePieceVisionIOInputs inputs) {
        // Update connection status based on whether an update has been seen in the last 250ms
        long lastChangeMicroseconds = latencySubscriber.getLastChange();
        inputs.connected = ((RobotController.getFPGATime() - lastChangeMicroseconds) / 1000) < 250;

        List<TargetObservation> targetObservations = new LinkedList<>();

        double[] rawDetections = rawDetectionsSubscriber.get();

        for (int i = 0; i < rawDetections.length / 12; i++) {
            int offset = i * 12;
            double tx = rawDetections[offset + 1];
            double ty = rawDetections[offset + 2];

            targetObservations.add(new TargetObservation(
                    // microseconds to seconds
                    lastChangeMicroseconds / (1000.0 * 1000.0),
                    Units.degreesToRadians(tx),
                    Units.degreesToRadians(ty)
            ));
        }

        inputs.targetObservations = targetObservations.toArray(TargetObservation[]::new);

        // Only needed when modifying values
        // Increases network traffic but recommended by Limelight
        //NetworkTableInstance.getDefault().flush();
    }
}
