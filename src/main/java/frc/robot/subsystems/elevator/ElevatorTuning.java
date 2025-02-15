package frc.robot.subsystems.elevator;

import frc.robot.util.PIDF;

import static frc.robot.subsystems.elevator.ElevatorConstants.gains;

public class ElevatorTuning {
    public static final PIDF.Tunable gainsTunable = gains.tunable("Elevator");
}