package frc.robot.subsystems.elevator;

import frc.robot.subsystems.rollers.RollersIOInputsAutoLogged;
import frc.robot.util.SubsystemBaseExt;
import lombok.RequiredArgsConstructor;

public class Elevator extends SubsystemBaseExt {
    @RequiredArgsConstructor
    public enum Goal {
        CHARACTERIZATION(0), // specially handled in periodic
        STOW(0),
        SCORE_L1(0),
        SCORE_L2(0),
        SCORE_L3(0),
        SCORE_L4(0),
        DESCORE_L2_L3(0),
        DESCORE_L3_L4(0);

        private final double setpointMeters;
    }

    private static final ElevatorIO io;
    private static final ElevatorIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

    private Goal goal = Goal.IDLE;
    private Double setpointMeters;

    private static Elevator instance;

    public static Elevator get() {
        if (instance == null)
            synchronized (Elevator.class) {
                instance = new Elevator();
            }

        return instance;
    }
}