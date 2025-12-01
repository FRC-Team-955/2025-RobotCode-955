package frc.robot.subsystems.intakeRoller;

import frc.lib.PIDF;

public class IntakeRollerTuning {

    public static final PIDF.Tunable intakeRollerPositionGainsTunable
            = IntakeRollerConstants.intakeRollerConfig.positionGains().tunable("Intake/Roller/Position");
    public static final PIDF.Tunable intakeRollerVelocityGainsTunable
            = IntakeRollerConstants.intakeRollerConfig.velocityGains().tunable("Intake/Roller/Velocity");


}
