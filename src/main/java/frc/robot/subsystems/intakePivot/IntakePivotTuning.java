package frc.robot.subsystems.intakePivot;

import frc.lib.PIDF;


public class IntakePivotTuning {

    public static final PIDF.Tunable IntakePivotGainsTunable
            = IntakePivotConstants.intakePivotConfig.gains()
            .tunable("Intake/Pivot");


}
