package frc.template.absoluteencoder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.util.Units;

public class AbsoluteEncoderIOCANcoder extends AbsoluteEncoderIO {
    private final CANcoder cancoder;

    private final StatusSignal<Double> absolutePosition;

    private double gearRatio = 1.0;
    private double offsetRad;

    public AbsoluteEncoderIOCANcoder(int canID) {
        cancoder = new CANcoder(canID);
        cancoder.getConfigurator().apply(new CANcoderConfiguration());

        absolutePosition = cancoder.getAbsolutePosition();
    }

    @Override
    public void updateInputs(AbsoluteEncoderIOInputs inputs) {
        inputs.isConnected = absolutePosition.refresh().getStatus().isOK();
        inputs.absolutePositionRad = Units.rotationsToRadians(absolutePosition.getValueAsDouble()) / gearRatio - offsetRad;
    }

    @Override
    public void setGearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
    }

    @Override
    public void setOffset(double offsetRad) {
        this.offsetRad = offsetRad;
    }
}
