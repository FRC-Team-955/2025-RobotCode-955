package frc.robot.subsystems.intakepivot;

import frc.lib.PIDF;

import static frc.robot.subsystems.intakepivot.IntakePivotConstants.gains;

public class IntakePivotTuning {
    public static final PIDF.Tunable gainsTunable = gains.tunable("IntakePivot/Gains");
}
