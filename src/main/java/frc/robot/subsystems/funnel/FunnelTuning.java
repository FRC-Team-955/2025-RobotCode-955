package frc.robot.subsystems.funnel;

import edu.wpi.first.math.util.Units;
import frc.robot.util.PIDF;
import frc.robot.util.network.LoggedTunableNumber;

import static frc.robot.subsystems.funnel.FunnelConstants.rollersConfig;

public class FunnelTuning {
    public static final PIDF.Tunable velocityGainsTunable = rollersConfig.velocityGains().tunable("Funnel/Belt/Velocity");

    public static final LoggedTunableNumber funnelIntakeGoalSetpoint =
            new LoggedTunableNumber("Funnel/Belt/Goal/FunnelIntake", Units.rotationsPerMinuteToRadiansPerSecond(500));
}
