package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;

/** Like the Command Scheduler, but all requirements are ignored and it just runs a command in the background */
public class BackgroundCommandScheduler {
    private Command currentCommand = null;
    private Command nextCommand = null;

    public BackgroundCommandScheduler() {
        Robot.registerBackgroundCommandScheduler(this);
    }

    public void periodicBeforeCommands() {
        if (currentCommand != null) {
            currentCommand.execute();

            // If finished, end it
            if (currentCommand.isFinished()) {
                end(false);
            }
        }
    }

    public void periodicAfterCommands() {
        if (nextCommand != null) {
            // End old command
            if (currentCommand != null) {
                end(true);
            }

            currentCommand = nextCommand;
            nextCommand = null;

            // Initialize and execute new command once
            currentCommand.initialize();
            currentCommand.execute();
        }
    }

    public Command scheduleInBackground(Command command) {
        return Commands.runOnce(() -> this.nextCommand = command);
    }

    public void cancelIfRunningInstantaneous() {
        if (currentCommand != null) {
            end(true);
        }
    }

    public Command cancelIfRunning() {
        return Commands.runOnce(this::cancelIfRunningInstantaneous);
    }

    public Command waitUntilFinish() {
        return Commands.waitUntil(() -> currentCommand == null);
    }

    private void end(boolean interrupted) {
        currentCommand.end(interrupted);
        currentCommand = null;
    }
}
