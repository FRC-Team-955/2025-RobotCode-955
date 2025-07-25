package frc.robot.util.subsystem;

public interface Periodic {
    default void periodicBeforeCommands() {
    }

    default void periodicAfterCommands() {
    }
}
