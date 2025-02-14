package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

public class ElevatorIOSim extends ElevatorIO {
    private static final DCMotor motors = DCMotor.getNEO(2);
    private final ElevatorSim sim = new ElevatorSim(
            motors,
            gearRatio,
            6.8,
            drumRadiusMeters,
            0,
            maxHeightMeters,
            true,
            0,
            0.0001, // position
            0.0001 // velocity, maybe
    );

    private final ElevatorFeedforward feedforward = gains.toElevatorFF();
    private final PIDController pidController = gains.toPID();

    private boolean closedLoop = true;
    private double setpointPositionRad;
    private double setpointVelocityRadPerSec;
    private double appliedVolts = 0.0;
    private double appliedAmps = 0.0;

    /** Used when calculating feedforward */
    private double lastVelocitySetpointRadPerSec = 0;


    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Run closed-loop control
        if (closedLoop) {
            if (useCurrentControl) {
                var pid = pidController.calculate(metersToRad(sim.getPositionMeters()), setpointPositionRad);
                var ff = feedforward.getKs() * Math.signum(pid) + feedforward.getKg();
                appliedAmps = pid;
                appliedVolts = ff + motors.getVoltage(motors.getTorque(appliedAmps), metersToRad(sim.getVelocityMetersPerSecond()));
            } else {
                var pid = pidController.calculate(metersToRad(sim.getPositionMeters()), setpointPositionRad);
                var ff = feedforward.calculateWithVelocities(lastVelocitySetpointRadPerSec, setpointVelocityRadPerSec);
                lastVelocitySetpointRadPerSec = setpointVelocityRadPerSec;
                appliedVolts = ff + pid;
            }
        } else {
            pidController.reset();
        }

        // Update simulation state
        sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
        sim.update(0.02);

        inputs.leaderConnected = true;
        inputs.leaderPositionRad = metersToRad(sim.getPositionMeters());
        inputs.leaderVelocityRadPerSec = metersToRad(sim.getVelocityMetersPerSecond());
        inputs.leaderAppliedVolts = appliedVolts;
        inputs.leaderCurrentAmps = Math.abs(sim.getCurrentDrawAmps());

        inputs.followerConnected = true;
        inputs.followerPositionRad = inputs.leaderPositionRad;
        inputs.followerVelocityRadPerSec = inputs.leaderVelocityRadPerSec;
        inputs.followerAppliedVolts = inputs.leaderAppliedVolts;
        inputs.followerCurrentAmps = inputs.leaderCurrentAmps;

        inputs.limitSwitchConnected = true;
        inputs.limitSwitchTriggered = sim.getPositionMeters() < Units.inchesToMeters(1);
    }

    @Override
    public void setBrakeMode(boolean enable) {
        System.out.println("Setting elevator break mode to " + enable);
    }

    @Override
    public void setOpenLoop(double output) {
        closedLoop = false;
        appliedVolts = output;
    }

    @Override
    public void setClosedLoop(double positionRad, double velocityRadPerSec) {
        closedLoop = true;
        setpointPositionRad = positionRad;
        setpointVelocityRadPerSec = velocityRadPerSec;
    }

    @Override
    public void setEncoder(double positionRad) {
        sim.setState(radToMeters(positionRad), sim.getVelocityMetersPerSecond());
    }
}
