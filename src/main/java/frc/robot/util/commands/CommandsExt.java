package frc.robot.util.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

public class CommandsExt {
    public static Command steppable(
            Trigger forwardTrigger,
            Trigger reverseTrigger,
            Command... commands
    ) {
        return new SteppableCommandGroup(forwardTrigger, reverseTrigger, commands);
    }

    public static Command waitUntilRequirements(BooleanSupplier isFinished, Subsystem... requirements) {
        return new WaitUntilRequirements(isFinished, requirements);
    }

    public static Command runOnceAndWaitUntil(
            Runnable initialize,
            BooleanSupplier isFinished,
            Subsystem... requirements
    ) {
        return new RunOnceWaitUntilRequirements(initialize, isFinished, requirements);
    }

    public static Command schedule(Command... commands) {
        return new ScheduleCommand(commands);
    }

    public static Command onlyIf(BooleanSupplier condition, Command onTrue) {
        return Commands.either(onTrue, Commands.none(), condition);
    }
}
