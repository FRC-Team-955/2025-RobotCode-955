package frc.robot.util.commands;

// package-private
/** Combination of Commands.runOnce and WaitUntilRequirements */
class RunOnceWaitUntilRequirements extends Command {
    private final Runnable initialize;
    private final BooleanSupplier isFinished;

    public WaitUntilRequirements(
        Runnable initialize,
        BooleanSupplier isFinished, 
        SubsystemBase requirements...
    ) {
        this.initialize = initialize;
        this.isFinished = isFinished;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        initialize.accept();
    }

    @Override
    public boolean isFinished() {
        return isFinished.getAsBoolean();
    }
}