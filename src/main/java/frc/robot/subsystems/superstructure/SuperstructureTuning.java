package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import frc.robot.util.network.LoggedTunableNumber;

public class SuperstructureTuning {
    public static final LoggedTunableNumber homeInitialMeters =
            new LoggedTunableNumber("Superstructure/HomeInitialMeters", Units.inchesToMeters(1.5));
    public static final LoggedTunableNumber homeFinalMeters =
            new LoggedTunableNumber("Superstructure/HomeFinalMeters", Units.inchesToMeters(2.5));

    public static final LoggedTunableNumber manualFunnelIntakeMeters =
            new LoggedTunableNumber("Superstructure/ManualFunnelIntakeMeters", 1.25);
}