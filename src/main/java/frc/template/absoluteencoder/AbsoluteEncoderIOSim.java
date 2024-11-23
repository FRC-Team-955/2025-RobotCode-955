package frc.template.absoluteencoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;

import static edu.wpi.first.units.Units.Radians;

public class AbsoluteEncoderIOSim extends AbsoluteEncoderIO {
    private final double positionRad;

    public AbsoluteEncoderIOSim() {
        positionRad = Math.random() * 2.0 * Math.PI;
    }

    public AbsoluteEncoderIOSim(Measure<Angle> minPosition, Measure<Angle> maxPosition) {
        positionRad = MathUtil.clamp(Math.random() * 2.0 * Math.PI, minPosition.in(Radians), maxPosition.in(Radians));
    }

    @Override
    public void updateInputs(AbsoluteEncoderIOInputs inputs) {
        inputs.isConnected = true;
        inputs.absolutePositionRad = positionRad;
    }
}
