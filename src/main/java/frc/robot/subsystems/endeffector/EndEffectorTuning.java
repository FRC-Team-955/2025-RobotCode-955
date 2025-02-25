package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.util.Units;
import frc.robot.util.PIDF;
import frc.robot.util.network.LoggedTunableNumber;

import static frc.robot.subsystems.endeffector.EndEffectorConstants.positionGains;
import static frc.robot.subsystems.endeffector.EndEffectorConstants.velocityGains;


public class EndEffectorTuning {
    public static final PIDF.Tunable positionGainsTunable = positionGains.tunable("EndEffector/Position");
    public static final PIDF.Tunable velocityGainsTunable = velocityGains.tunable("EndEffector/Velocity");

    public static final LoggedTunableNumber funnelIntakeGoalSetpoint =
            new LoggedTunableNumber("EndEffector/Goal/FunnelIntake", Units.rotationsPerMinuteToRadiansPerSecond(350));
    public static final LoggedTunableNumber orientCoralGoalSetpoint =
            new LoggedTunableNumber("EndEffector/Goal/OrientCoral", Units.rotationsPerMinuteToRadiansPerSecond(100));
    public static final LoggedTunableNumber scoreCoralGoalSetpoint =
            new LoggedTunableNumber("EndEffector/Goal/ScoreCoral", Units.rotationsPerMinuteToRadiansPerSecond(100));
    public static final LoggedTunableNumber descoreAlgaeGoalSetpoint =
            new LoggedTunableNumber("EndEffector/Goal/DescoreAlgae", Units.rotationsPerMinuteToRadiansPerSecond(-300));
}