package frc.robot.subsystems.endeffector;

import frc.robot.util.PIDF;

import static frc.robot.subsystems.endeffector.EndEffectorConstants.positionGains;
import static frc.robot.subsystems.endeffector.EndEffectorConstants.velocityGains;


public class EndEffectorTuning {
    public static final PIDF.Tunable positionGainsTunable = positionGains.tunable("EndEffector/Position");
    public static final PIDF.Tunable velocityGainsTunable = velocityGains.tunable("EndEffector/Velocity");
}