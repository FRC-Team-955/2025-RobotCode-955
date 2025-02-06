package frc.robot.util.commands;

// package-private
    /** Like WaitUntil, but requires a subsystem and doesn't run while disabled. */
class WaitUntilRequirements extends Command {
    private final BooleanSupplier isFinished;

    public WaitUntilRequirements(BooleanSupplier isFinished, SubsystemBase requirements...) {
        this.isFinished = isFinished;
        addRequirements(requirements);
    }

    @Override
    public boolean isFinished() {
        return isFinished.getAsBoolean();
    }
}