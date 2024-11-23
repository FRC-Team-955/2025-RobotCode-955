package frc.robot.subsystems.shooter;

import com.pathplanner.lib.util.PIDConstants;
import org.littletonrobotics.junction.AutoLog;

public class ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public boolean hasNote = false;

        public double pivotPositionRad = 0.0;
        public double pivotVelocityRadPerSec = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;

        public double feedPositionRad = 0.0;
        public double feedVelocityRadPerSec = 0.0;
        public double feedAppliedVolts = 0.0;
        public double feedCurrentAmps = 0.0;

        public double flywheelTopPositionRad = 0.0;
        public double flywheelTopVelocityRadPerSec = 0.0;
        public double flywheelTopAppliedVolts = 0.0;
        public double flywheelTopCurrentAmps = 0.0;

        public double flywheelBottomPositionRad = 0.0;
        public double flywheelBottomVelocityRadPerSec = 0.0;
        public double flywheelBottomAppliedVolts = 0.0;
        public double flywheelBottomCurrentAmps = 0.0;
    }

    public void updateInputs(ShooterIOInputs inputs) {
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

    public void flywheelsSetVoltage(double volts) {
    }

    public void flywheelsSetSetpoint(double setpointVelocityRadPerSec, double ffTopVolts, double ffBottomVolts) {
    }

    public void flywheelsSetBrakeMode(boolean enabled) {
    }

    public void flywheelsTopConfigurePID(PIDConstants pidConstants) {
    }

    public void flywheelsBottomConfigurePID(PIDConstants pidConstants) {
    }
}
