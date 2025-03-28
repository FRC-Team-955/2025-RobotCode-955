package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import frc.robot.util.network.LoggedTunableNumber;

public class SuperstructureTuning {
    public static final LoggedTunableNumber funnelIntakeInitialMeters =
            new LoggedTunableNumber("Superstructure/FunnelIntakeInitialMeters", Units.inchesToMeters(1));
    public static final LoggedTunableNumber funnelIntakeHomeMeters =
            new LoggedTunableNumber("Superstructure/FunnelIntakeHomeMeters", Units.inchesToMeters(2));
}