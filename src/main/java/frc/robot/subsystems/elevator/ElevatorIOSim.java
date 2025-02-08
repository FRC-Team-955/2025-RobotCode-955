package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

public class ElevatorIOSim extends ElevatorIO {
    private final ElevatorSim sim = new ElevatorSim(
            DCMotor.getNEO(2),
            gearRatio,
            6.8,
            drumRadiusMeters,
            0,
            Units.inchesToMeters(66.622),
            true,
            0
    );

    private final ElevatorFeedforward feedforward = gains.toElevatorFF();
    private final PIDController pidController = gains.toPID();

    private boolean closedLoop = true;
    private double setpointPositionRad;
    private double setpointVelocityRadPerSec;
    private double appliedVolts = 0.0;


    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Run closed-loop control
        if (closedLoop) {
            var pid = pidController.calculate(metersToRad(sim.getPositionMeters()), setpointPositionRad);
            var ff = feedforward.calculateWithVelocities(
                    metersToRad(sim.getVelocityMetersPerSecond()),
                    setpointVelocityRadPerSec
            );
            appliedVolts = ff + pid;
        } else {
            pidController.reset();
        }

        // Update simulation state
        sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
        sim.update(0.02);

        inputs.connected = true;
        inputs.positionRad = metersToRad(sim.getPositionMeters());
        inputs.velocityRadPerSec = metersToRad(sim.getVelocityMetersPerSecond());
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
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
}
