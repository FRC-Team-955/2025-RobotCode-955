package frc.robot.util.subsystem;

import frc.robot.Robot;

public class VirtualSubsystem {
    public VirtualSubsystem() {
        Robot.registerVirtualSubsystem(this);
    }

    public void periodicBeforeCommands() {
    }

    public void periodicAfterCommands() {
    }
}

