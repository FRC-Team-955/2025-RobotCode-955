package frc.robot.subsystems.elevator;

import frc.robot.subsystems.rollers.RollersIOInputsAutoLogged;
import frc.robot.util.SubsystemBaseExt;

public class Elevator extends SubsystemBaseExt {
    public enum Goal {
    }

    private static final ElevatorIO io;
    private static final ElevatorIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

    private Goal goal = Goal.IDLE;
    private double setpointRad;

    private static Elevator instance;

    public static Elevator get() {
        if (instance == null)
            synchronized (Elevator.class) {
                instance = new Elevator();
            }

        return instance;
    }
}