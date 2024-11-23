package frc.robot.subsystems.intake;

import com.pathplanner.lib.util.PIDConstants;
import org.littletonrobotics.junction.AutoLog;

public class IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean hasNote = false;

        public double pivotPositionRad = 0.0;
        public double pivotVelocityRadPerSec = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;

        public double feedPositionRad = 0.0;
        public double feedVelocityRadPerSec = 0.0;
        public double feedAppliedVolts = 0.0;
        public double feedCurrentAmps = 0.0;
    }

    public void updateInputs(IntakeIOInputs inputs) {
    }

    public void pivotSetVoltage(double volts) {
    }

    public void pivotSetSetpoint(double setpointPositionRad, double ffVolts) {
    }

    public void pivotSetPosition(double currentPositionRad) {
    }

    public void pivotSetBrakeMode(boolean enabled) {
    }

    public void pivotConfigurePID(PIDConstants pidConstants) {
    }

    public void feedSetVoltage(double volts) {
    }

    public void feedSetSetpoint(double setpointVelocityRadPerSec, double ffVolts) {
    }

    public void feedSetBrakeMode(boolean enabled) {
    }

    public void feedConfigurePID(PIDConstants pidConstants) {
    }
}
