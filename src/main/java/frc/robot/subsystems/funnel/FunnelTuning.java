package frc.robot.subsystems.funnel;

import edu.wpi.first.math.util.Units;
import frc.robot.util.PIDF;
import frc.robot.util.network.LoggedTunableNumber;

import static frc.robot.subsystems.funnel.FunnelConstants.beltConfig;

public class FunnelTuning {
    public static final PIDF.Tunable velocityGainsTunable = beltConfig.velocityGains().tunable("Funnel/Belt/Velocity");

    public static final LoggedTunableNumber intakeGoalSetpoint =
            new LoggedTunableNumber("Funnel/Belt/Goal/Intake", Units.rotationsPerMinuteToRadiansPerSecond(700));
    public static final LoggedTunableNumber ejectGoalSetpoint =
            new LoggedTunableNumber("Funnel/Belt/Goal/Eject", Units.rotationsPerMinuteToRadiansPerSecond(1000));
}
