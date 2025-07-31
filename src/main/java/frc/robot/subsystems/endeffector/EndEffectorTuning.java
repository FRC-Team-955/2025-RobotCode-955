package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.util.Units;
import frc.robot.util.PIDF;
import frc.robot.util.network.LoggedTunableNumber;

import static frc.robot.subsystems.endeffector.EndEffectorConstants.rollersConfig;


public class EndEffectorTuning {
    public static final PIDF.Tunable positionGainsTunable = rollersConfig.positionGains().tunable("EndEffector/Position");
    public static final PIDF.Tunable velocityGainsTunable = rollersConfig.velocityGains().tunable("EndEffector/Velocity");

    public static final LoggedTunableNumber funnelIntakeGoalSetpoint =
            new LoggedTunableNumber("EndEffector/Goal/FunnelIntake", Units.rotationsPerMinuteToRadiansPerSecond(500));
    public static final LoggedTunableNumber funnelIntakeManualGoalSetpoint =
            new LoggedTunableNumber("EndEffector/Goal/FunnelIntakeManual", Units.rotationsPerMinuteToRadiansPerSecond(200));
    public static final LoggedTunableNumber scoreCoralGoalSetpoint =
            new LoggedTunableNumber("EndEffector/Goal/ScoreCoral", Units.rotationsPerMinuteToRadiansPerSecond(300));
    public static final LoggedTunableNumber scoreCoralL1GoalSetpoint =
            new LoggedTunableNumber("EndEffector/Goal/ScoreCoralL1", Units.rotationsPerMinuteToRadiansPerSecond(600));
    public static final LoggedTunableNumber descoreAlgaeGoalSetpoint =
            new LoggedTunableNumber("EndEffector/Goal/DescoreAlgae", Units.rotationsPerMinuteToRadiansPerSecond(-600));
    public static final LoggedTunableNumber ejectGoalSetpoint =
            new LoggedTunableNumber("EndEffector/Goal/Eject", Units.rotationsPerMinuteToRadiansPerSecond(450));
    public static final LoggedTunableNumber zeroCoralGoalSetpoint =
            new LoggedTunableNumber("EndEffector/Goal/ZeroCoral", Units.rotationsPerMinuteToRadiansPerSecond(-600));
}