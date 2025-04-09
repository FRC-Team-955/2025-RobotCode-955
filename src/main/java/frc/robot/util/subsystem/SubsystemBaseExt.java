package frc.robot.util.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public abstract class SubsystemBaseExt extends SubsystemBase implements SubsystemExt {
    /** Lower = will be run first */
    public final int periodicPriority;

    public SubsystemBaseExt(int periodicPriority) {
        this.periodicPriority = periodicPriority;
        Robot.registerExtendedSubsystem(this);
    }

    public SubsystemBaseExt(String name, int periodicPriority) {
        super(name);
        this.periodicPriority = periodicPriority;
        Robot.registerExtendedSubsystem(this);
    }

    @Override
    // did you know you can have final methods in java? neither did I
    public final void periodic() {
        // periodicBeforeCommands is run by Robot
    }
}
