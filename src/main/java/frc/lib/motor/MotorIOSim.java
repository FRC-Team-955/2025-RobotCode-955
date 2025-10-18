package frc.lib.motor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.PIDF;

public class MotorIOSim extends MotorIO {
    private final DCMotorSim motorSim;
    private PIDController positionPid;
    private PIDController velocityPid;
    private SimpleMotorFeedforward velocityFeedForward;

    private double appliedVolts;
    private boolean closedLoop = true;
    private boolean positionControl = false;
    private double ffVolts;

    public MotorIOSim(DCMotor motor, double kV, double kA, PIDF positionGains, PIDF velocityGains) {
        motorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(kV, kA),
                motor,
                0.004
        );

        positionPid = positionGains.toPID();
        velocityFeedForward = velocityGains.toSimpleFF();
        velocityPid = velocityGains.toPID();
    }

    public MotorIOSim(double gearRatio, double JkgMetersSquared, DCMotor motor, PIDF positionGains, PIDF velocityGains) {
        motorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(motor, JkgMetersSquared, gearRatio),
                motor,
                0.004,
                0.0
        );

        velocityFeedForward = velocityGains.toSimpleFF();
        positionPid = positionGains.toPID();
        velocityPid = velocityGains.toPID();
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        if (closedLoop) {
            if (positionControl) {
                appliedVolts = positionPid.calculate(motorSim.getAngularPositionRad());
            } else {
                appliedVolts = velocityPid.calculate(motorSim.getAngularVelocityRadPerSec()) + ffVolts;
            }
        }

        motorSim.setInputVoltage(appliedVolts);
        motorSim.update(0.02);

        inputs.connected = true;
        inputs.positionRad = motorSim.getAngularPositionRad();
        inputs.velocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = Math.abs(motorSim.getCurrentDrawAmps());
    }

    @Override
    public void setPositionPIDF(PIDF newGains) {
        System.out.println("Setting roller position gains");
        positionPid = newGains.toPID();
    }

    @Override
    public void setVelocityPIDF(PIDF newGains) {
        System.out.println("Setting roller velocity gains");
        velocityFeedForward = newGains.toSimpleFF();
        velocityPid = newGains.toPID();
    }

    @Override
    public void setBrakeMode(boolean enable) {
        System.out.println("Setting rollers brake mode to " + enable);
    }

    @Override
    public void setRequest(RequestType type, double value) {
        switch (type) {
            case VoltageVolts -> {
                appliedVolts = value;
                closedLoop = false;
            }
            case PositionRad -> {
                closedLoop = true;
                positionControl = true;
                positionPid.setSetpoint(value);
            }
            case VelocityRadPerSec -> {
                closedLoop = true;
                positionControl = false;
                ffVolts = velocityFeedForward.calculate(value);
                velocityPid.setSetpoint(value);
            }
        }
    }
}
