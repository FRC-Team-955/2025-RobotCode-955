package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public abstract class SubsystemBaseExt extends SubsystemBase implements SubsystemExt {
    public SubsystemBaseExt() {
        Robot.registerExtendedSubsystem(this);
    }

    @Override
    // did you know you can have final methods in java? neither did I
    public final void periodic() {
        this.periodicBeforeCommands();
    }
}
