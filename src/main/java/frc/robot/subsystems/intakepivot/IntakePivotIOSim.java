package frc.robot.subsystems.intakepivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.PIDF;
import frc.lib.Util;
import frc.lib.motor.MotorIO;
import frc.lib.motor.RequestType;

public class IntakePivotIOSim extends MotorIO {
    private static final double voltageLimit = 8.0;
    private static final double currentLimit = 40.0;
    private final DCMotor motor;

    private final SingleJointedArmSim armSim;
    private PIDController pid;
    private ArmFeedforward ff;

    private double appliedVolts;
    private boolean closedLoop = true;

    // If using physical values, JKgMetersSquared is the moment of inertia J of the flywheel
    public IntakePivotIOSim(double gearRatio, DCMotor motor, PIDF gains) {
        this.motor = motor.withReduction(gearRatio);
        armSim = new SingleJointedArmSim(
                motor,
                gearRatio,
                1.4,
                0.3,
                0,
                Units.degreesToRadians(80),
                true,
                Units.degreesToRadians(80),
                0.001,
                0.0001
        );

        pid = gains.toPID();
        ff = gains.toArmFF();
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        if (closedLoop) {
            appliedVolts = pid.calculate(armSim.getAngleRads()) + ff.calculate(armSim.getAngleRads(), 0);
            appliedVolts = MathUtil.clamp(appliedVolts, -voltageLimit, voltageLimit);

            // TODO: https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/27ef554d86a62d8dba1a361cc5eca8919d4be1f9/project/src/main/java/org/ironmaple/simulation/motorsims/SimulatedMotorController.java#L65-L83
            double maxTorqueForCurrentLimit = motor.getTorque(currentLimit);
            double maxVoltsForCurrentLimit = motor.getVoltage(
                    Math.copySign(maxTorqueForCurrentLimit, appliedVolts),
                    armSim.getVelocityRadPerSec()
            );
            // maxVoltsForCurrentLimit will be
            if (appliedVolts > 0) {
                appliedVolts = Math.min(appliedVolts, maxVoltsForCurrentLimit);
            } else {
                appliedVolts = Math.max(appliedVolts, maxVoltsForCurrentLimit);
            }
            appliedVolts = Math.copySign(maxVoltsForCurrentLimit, Timer.getTimestamp() % 1 - 0.5);
        }

        armSim.setInputVoltage(appliedVolts);

        armSim.update(0.02);

        inputs.connected = true;
        inputs.positionRad = armSim.getAngleRads();
        inputs.velocityRadPerSec = armSim.getVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = Math.abs(armSim.getCurrentDrawAmps());
    }

    @Override
    public void setPositionPIDF(PIDF newGains) {
        System.out.println("Setting intake pivot position gains");
        pid = newGains.toPID();
        ff = newGains.toArmFF();
    }

    @Override
    public void setVelocityPIDF(PIDF newGains) {
        Util.error("Intake pivot should only set position PIDF");
    }

    @Override
    public void setBrakeMode(boolean enable) {
        System.out.println("Setting intake pivot brake mode to " + enable);
    }

    @Override
    public void setRequest(RequestType type, double value) {
        switch (type) {
            case PositionRad -> {
                closedLoop = true;
                pid.setSetpoint(value);
            }
            case VoltageVolts -> {
                closedLoop = false;
                appliedVolts = 0.0;
            }
            default -> Util.error("Intake pivot should only use PositionRad");
        }
    }
}
