package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SubsystemExt extends Subsystem {
    default void periodicBeforeCommands() {
    }

    default void periodicAfterCommands() {
    }

    default void onCommandEnd() {
    }
}
