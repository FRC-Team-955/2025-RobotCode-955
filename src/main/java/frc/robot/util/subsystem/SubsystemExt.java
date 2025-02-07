package frc.robot.util.subsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.commands.CommandsExt;

import java.util.function.BooleanSupplier;

public interface SubsystemExt extends Subsystem {
    default void periodicBeforeCommands() {
    }

    default void periodicAfterCommands() {
    }

//    default void onCommandEnd() {
//    }

    default Command waitUntil(BooleanSupplier isFinished) {
        return CommandsExt.waitUntilRequirements(isFinished, this);
    }

    default Command runOnceAndWaitUntil(Runnable initialize, BooleanSupplier isFinished) {
        return CommandsExt.runOnceAndWaitUntil(initialize, isFinished, this);
    }
}
