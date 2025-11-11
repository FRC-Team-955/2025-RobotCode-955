package frc.robot.subsystems.intakerollers;

import frc.lib.PIDF;
import frc.robot.subsystems.intakepivot.IntakeConstants;

public class IntakeRollersTuning {
//    public static final PIDF.Tunable moduleIntakeRollersPositionGainsTunable = IntakeRollersConstants.intakeRollersConfig.positionGains().tunable("Intake/Rollers/Position");
    public static final PIDF.Tunable moduleIntakeRollersVelocityGainsTunable = IntakeRollersConstants.intakeRollersConfig.velocityGains().tunable("Intake/Rollers");
}
