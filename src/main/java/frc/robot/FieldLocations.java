package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.Supplier;

public final class FieldLocations {
    public static Supplier<Translation2d> SPEAKER = () -> Util.flipIfNeeded(new Translation2d(0, 5.55));
}
