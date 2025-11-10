package frc.robot.subsystems.intake;

import frc.lib.PIDF;

public class IntakeTuning {
    public static final PIDF.Tunable moduleIntakeGainsTunable = IntakeConstants.intakeConfig.gains().tunable("Intake/Pivot");
}
