package frc.robot.subsystems.gamePieceVision;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

public class GamePieceVisionIOLimelight extends GamePieceVisionIO {

    private final IntegerSubscriber tvSubscriber;
    private final CANrange canRange = new CANrange(0);
    private final DoubleSubscriber txSubscriber;
    private final DoubleSubscriber tySubscriber;
    private final DoubleSubscriber taSubscriber;
    private final DoubleSubscriber latencySubscriber;

    private final Transform3d camToRobot = new Transform3d(
            Units.inchesToMeters(0.0), Units.inchesToMeters(-10), Units.inchesToMeters(30.0),
            // Rotation order matters
            new Rotation3d(0.0, Units.degreesToRadians(30), 0.0)
                    .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(-90)))
    );

    public GamePieceVisionIOLimelight(String name) {
        var table = NetworkTableInstance.getDefault().getTable(name);
        tvSubscriber = table.getIntegerTopic("tv").subscribe(0);
        txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
        tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
        taSubscriber = table.getDoubleTopic("ta").subscribe(0.0);
        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);


    }

    public void updateInputs(GamePieceVisionIOInputs inputs) {
        inputs.connected = ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

        inputs.visible = tvSubscriber.get() == 1;
    }
}
