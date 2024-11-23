package frc.template.absoluteencoder;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Radians;

public final class ExampleSubsystemUsage extends SubsystemBase {
    private final AbsoluteEncoderIOInputsAutoLogged absoluteEncoderInputs = new AbsoluteEncoderIOInputsAutoLogged();
    private final AbsoluteEncoderIO absoluteEncoderIO;

    /**
     * @param absoluteEncoderGearRatio >1 means a reduction, <1 means a upduction
     */
    private ExampleSubsystemUsage(
            AbsoluteEncoderIO absoluteEncoderIO,
            Double absoluteEncoderGearRatio,
            Measure<Angle> absoluteEncoderOffset
    ) {
        this.absoluteEncoderIO = absoluteEncoderIO;

        if (absoluteEncoderGearRatio != null) absoluteEncoderIO.setGearRatio(absoluteEncoderGearRatio);
        absoluteEncoderIO.setOffset(absoluteEncoderOffset.in(Radians));
    }

    /**
     * Inputs will not be logged by this class. You must log the returned inputs yourself.
     */
    @Override
    public void periodic() {
        absoluteEncoderIO.updateInputs(absoluteEncoderInputs);
        Logger.processInputs("Inputs/ExampleSubsystem", absoluteEncoderInputs);
    }

    public boolean isConnected() {
        return absoluteEncoderInputs.isConnected;
    }

    public Measure<Angle> getPosition() {
        return Radians.of(absoluteEncoderInputs.absolutePositionRad);
    }
}
