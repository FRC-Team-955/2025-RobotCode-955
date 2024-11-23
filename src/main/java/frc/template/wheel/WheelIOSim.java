package frc.template.wheel;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class WheelIOSim extends WheelIO {
    private FlywheelSim sim;
    private PIDController pid;

    private double appliedVolts;
    private boolean closedLoop = true;
    private double ffVolts;

    private final DCMotor motor;
    private final double jKgMetersSquared;

    public WheelIOSim(DCMotor motor) {
        this(motor, 0.004);
    }

    public WheelIOSim(DCMotor motor, double jKgMetersSquared) {
        this.motor = motor;
        this.jKgMetersSquared = jKgMetersSquared;
    }

    @Override
    public void updateInputs(WheelIOInputs inputs) {
        if (closedLoop) {
            appliedVolts = pid.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts;
            sim.setInputVoltage(appliedVolts);
        }

        sim.update(0.02);

        inputs.positionRad = 0.0;
        inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
        sim.setInputVoltage(appliedVolts);
        closedLoop = false;
    }

    @Override
    public void setSetpoint(double setpointVelocityRadPerSec, double ffVolts) {
        this.ffVolts = ffVolts;
        pid.setSetpoint(setpointVelocityRadPerSec);
        closedLoop = true;
    }

    @Override
    public void configurePID(PIDConstants pidConstants) {
        pid = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD);
        pid.setIZone(pidConstants.iZone);
    }

    @Override
    public void setGearRatio(double gearRatio) {
        sim = new FlywheelSim(motor, gearRatio, jKgMetersSquared);
    }
}
