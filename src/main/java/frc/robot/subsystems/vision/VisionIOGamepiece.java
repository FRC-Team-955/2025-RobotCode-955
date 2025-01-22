package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.RobotController;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

public class VisionIOGamepiece extends VisionIO {
    private final DoubleSubscriber latencySubscriber;
    private final DoubleSubscriber txSubscriber;
    private final DoubleSubscriber tySubscriber;
    private final StringSubscriber tclassSubscriber;
    private final IntegerSubscriber tvSubscriber;

    public VisionIOGamepiece(String name) {
        var table = NetworkTableInstance.getDefault().getTable(name);

        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
        tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
        tclassSubscriber = table.getStringTopic("tclass").subscribe("");
        tvSubscriber = table.getIntegerTopic("tv").subscribe(0);
    }


    @Override
    public void updateInputs(VisionIO.VisionIOInputs inputs) {
        // Update connection status based on whether an update has been seen in the last 250ms
        inputs.connected =
                ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

        // Update target observation
        inputs.latestTargetObservation =
                new VisionIO.TargetObservation(
                        Rotation2d.fromDegrees(txSubscriber.get()),
                        Rotation2d.fromDegrees(tySubscriber.get()),
                        tvSubscriber.get() == 1
                );

        NetworkTableInstance.getDefault()
                .flush(); // Increases network traffic but recommended by Limelight
    }
}
