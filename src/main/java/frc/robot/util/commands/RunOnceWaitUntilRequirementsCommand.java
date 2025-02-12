package frc.robot.util.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.BooleanSupplier;

/** Combination of Commands.runOnce and WaitUntilRequirements */
/* package-private */ class RunOnceWaitUntilRequirementsCommand extends Command {
    private final Runnable initialize;
    private final BooleanSupplier isFinished;

    public RunOnceWaitUntilRequirementsCommand(
            Runnable initialize,
            BooleanSupplier isFinished,
            Subsystem... requirements
    ) {
        this.initialize = initialize;
        this.isFinished = isFinished;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        initialize.run();
    }

    @Override
    public boolean isFinished() {
        return isFinished.getAsBoolean();
    }
}