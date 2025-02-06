package frc.robot.util.commands;

public class CommandsExt {
    public static Comamnd steppable(
        Trigger forwardTrigger, 
        Trigger reverseTrigger, 
        Command... commands
    ) {
        return new SteppableCommandGroup(forwardTrigger, reverseTrigger, commands);
    }
    
    public static Command waitUntilRequirements(BooleanSupplier isFinished, SubsystemBase requirements...) {
        return new WaitUntilRequirements(isFinished, requirements);
    }

    public static Command runOnceAndWaitUntil(
        Runnable initialize,
        BooleanSupplier isFinished,
        SubsystemBase requirements...
    ) {
        return new RunOnceWaitUntilRequirements(initialize, isFinished, requirements);
    }
}
