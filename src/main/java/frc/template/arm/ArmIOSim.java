package frc.template.arm;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim extends ArmIO {
    private SingleJointedArmSim sim;
    private PIDController pid;

    private double appliedVolts;
    private boolean closedLoop = true;
    private double ffVolts;

    private final DCMotor motor;
    private final double armLength;
    private final double jKgMetersSquared;

    /**
     * @param jKgMetersSquared This can be calculated in Onshape with the "Display mass properties" button in the bottom left. If unsure, you can use 0.1 temporarily.
     */
    public ArmIOSim(DCMotor motor, double armLengthMeters, double jKgMetersSquared) {
        this.motor = motor;
        this.armLength = armLengthMeters;
        this.jKgMetersSquared = jKgMetersSquared;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        if (closedLoop) {
            appliedVolts = pid.calculate(sim.getAngleRads()) + ffVolts;
            sim.setInputVoltage(appliedVolts);
        }

        sim.update(0.02);

        inputs.positionRad = sim.getAngleRads();
        inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
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
    public void setSetpoint(double setpointPositionRad, double ffVolts) {
        this.ffVolts = ffVolts;
        pid.setSetpoint(setpointPositionRad);
        closedLoop = true;
    }

    @Override
    public void setPosition(double currentPositionRad) {
        sim.setState(currentPositionRad, 0);
    }

    @Override
    public void configurePID(PIDConstants pidConstants) {
        pid = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD);
        pid.setIZone(pidConstants.iZone);
    }

    @Override
    public void setGearRatio(double gearRatio) {
        sim = new SingleJointedArmSim(
                motor,
                gearRatio,
                jKgMetersSquared,
                armLength,
                -Math.PI * 2,
                Math.PI * 2,
                true,
                0.0
        );
    }
}
