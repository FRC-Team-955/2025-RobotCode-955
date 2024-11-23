package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;

public class GyroIOPigeon2 extends GyroIO {
    private final Pigeon2 pigeon;
    private final StatusSignal<Double> yaw;
    private final StatusSignal<Double> yawVelocity;

    public GyroIOPigeon2(int canID) {
        pigeon = new Pigeon2(canID);
        yaw = pigeon.getYaw();
        yawVelocity = pigeon.getAngularVelocityZWorld();

        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(100.0);
        yawVelocity.setUpdateFrequency(100.0);
        pigeon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.isConnected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        inputs.yawPositionRad = Units.degreesToRadians(yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    }
}
