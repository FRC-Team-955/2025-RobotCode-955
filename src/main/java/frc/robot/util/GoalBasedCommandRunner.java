package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

/**
 * A dummy subsystem that can be put in a field of a parent subsystem.
 * <p>
 * Automatically manages cancelling the current command if the goal is changed and allows
 * scheduling commands for the subsystem without interrupting the current goal command.
 */
public class GoalBasedCommandRunner<T> extends SubsystemBase {
    private final Supplier<T> goalSupplier;
    private T lastGoal;

    public GoalBasedCommandRunner(String name, Supplier<T> goalSupplier) {
        super(name);
        this.goalSupplier = goalSupplier;
        lastGoal = goalSupplier.get();
    }

    @Override
    public void periodic() {
        var currentGoal = goalSupplier.get();

        // Goal change
        if (currentGoal != lastGoal) {
            // If we had a command scheduled
            var currentCommand = getCurrentCommand();
            if (currentCommand != null) {
                // Cancel it
                currentCommand.cancel();
            }

            lastGoal = currentGoal;
        }
    }
}