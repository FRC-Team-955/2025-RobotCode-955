package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.Logger;

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
    private final ProfiledPIDController pidController = gains.toProfiledPID(
            new TrapezoidProfile.Constraints(
                    metersToRad(maxVelocityMetersPerSecond),
                    metersToRad(maxAccelerationMetersPerSecondSquared)
            )
    );

    private boolean closedLoop = true;
    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Run closed-loop control
        if (closedLoop) {
            var pid = pidController.calculate(metersToRad(sim.getPositionMeters()));
            // TODO: not replayable - do Trapezoid profile in subsystem?
            Logger.recordOutput("Elevator/Setpoint/VelocityRadPerSec", pidController.getSetpoint().velocity);
            var ff = feedforward.calculateWithVelocities(
                    metersToRad(sim.getVelocityMetersPerSecond()),
                    pidController.getSetpoint().velocity
            );
            appliedVolts = ff + pid;
        } else {
            pidController.reset(
                    metersToRad(sim.getPositionMeters()),
                    metersToRad(sim.getVelocityMetersPerSecond())
            );
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
    public void setPosition(double positionRad, double maxVelocityRadPerSec) {
        closedLoop = true;
        pidController.setGoal(new TrapezoidProfile.State(positionRad, 0));
        pidController.setConstraints(new TrapezoidProfile.Constraints(maxVelocityRadPerSec, metersToRad(maxAccelerationMetersPerSecondSquared)));
    }
}
