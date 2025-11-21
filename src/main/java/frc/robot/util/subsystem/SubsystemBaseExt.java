package frc.robot.util.subsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class SubsystemBaseExt extends SubsystemBase implements SubsystemExt {
    public SubsystemBaseExt() {
        Robot.registerExtendedSubsystem(this);
    }

    public SubsystemBaseExt(String name) {
        super(name);
        Robot.registerExtendedSubsystem(this);
    }

    @Override
    public final void periodic() {
        this.periodicBeforeCommands();
    }
}
