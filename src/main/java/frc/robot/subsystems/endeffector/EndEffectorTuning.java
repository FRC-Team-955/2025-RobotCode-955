package frc.robot.subsystems.endeffector;

import frc.robot.util.PIDF;

import static frc.robot.subsystems.endeffector.EndEffectorConstants.positionGains;


public class EndEffectorTuning {
    public static final PIDF.Tunable positionGainsTunable = positionGains.tunable("EndEffector/Position");
}