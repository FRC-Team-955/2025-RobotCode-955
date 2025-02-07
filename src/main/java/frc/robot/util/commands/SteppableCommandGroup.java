// MIT License
//
// Copyright (c) 2024 FRC 6328
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

package frc.robot.util.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalInt;

/* package-private */ class SteppableCommandGroup extends Command {
    private final Trigger forwardTrigger;
    private final Trigger reverseTrigger;
    private final List<Command> commands = new ArrayList<>();
    private boolean forwardTriggerState = false;
    private boolean reverseTriggerState = false;
    private int currentCommandIndex = -1;
    private boolean commandFinished = false;
    private boolean runsWhenDisabled = true;
    private InterruptionBehavior interruptionBehavior = InterruptionBehavior.kCancelIncoming;

    public SteppableCommandGroup(
            Trigger forwardTrigger,
            Trigger reverseTrigger,
            Command... commands
    ) {
        this.forwardTrigger = forwardTrigger;
        this.reverseTrigger = reverseTrigger;
        addCommands(commands);
    }

    public final void addCommands(Command... commands) {
        if (currentCommandIndex != -1) {
            throw new IllegalStateException(
                    "Commands cannot be added to a composition while it's running");
        }

        CommandScheduler.getInstance().registerComposedCommands(commands);

        for (Command command : commands) {
            this.commands.add(command);
            addRequirements(command.getRequirements());
            runsWhenDisabled &= command.runsWhenDisabled();
            if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
                interruptionBehavior = InterruptionBehavior.kCancelSelf;
            }
        }
    }

    @Override
    public final void initialize() {
        currentCommandIndex = 0;
        commandFinished = false;
        if (!commands.isEmpty()) {
            commands.get(0).initialize();
        }

        forwardTriggerState = forwardTrigger.getAsBoolean();
        reverseTriggerState = reverseTrigger.getAsBoolean();
    }

    @Override
    public final void execute() {
        if (commands.isEmpty()) {
            return;
        }

        Command currentCommand = commands.get(currentCommandIndex);
        if (currentCommand.isFinished() && !commandFinished) {
            currentCommand.end(false);
            commandFinished = true;
        }
        if (!commandFinished) {
            currentCommand.execute();
        }

        boolean stepForward = forwardTrigger.getAsBoolean() && !forwardTriggerState;
        boolean stepReverse = reverseTrigger.getAsBoolean() && !reverseTriggerState;
        forwardTriggerState = forwardTrigger.getAsBoolean();
        reverseTriggerState = reverseTrigger.getAsBoolean();

        int step = stepForward ? 1 : (stepReverse ? -1 : 0);
        int newCommandIndex = MathUtil.clamp(currentCommandIndex + step, 0, commands.size() - 1);
        if (newCommandIndex != currentCommandIndex) {
            if (!commandFinished) {
                currentCommand.end(true);
            }
            commands.get(newCommandIndex).initialize();
            currentCommandIndex = newCommandIndex;
            commandFinished = false;
        }
    }

    @Override
    public final void end(boolean interrupted) {
        if (interrupted && !commandFinished && !commands.isEmpty() && currentCommandIndex > -1) {
            commands.get(currentCommandIndex).end(true);
        }
        currentCommandIndex = -1;
    }

    @Override
    public boolean runsWhenDisabled() {
        return runsWhenDisabled;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return interruptionBehavior;
    }

    public OptionalInt getCurrentCommandIndex() {
        if (currentCommandIndex == -1) {
            return OptionalInt.empty();
        } else {
            return OptionalInt.of(currentCommandIndex);
        }
    }
}