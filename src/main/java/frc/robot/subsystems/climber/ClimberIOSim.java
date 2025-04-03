package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static frc.robot.subsystems.climber.ClimberConstants.gearRatio;

public class ClimberIOSim extends ClimberIO {
    private final DCMotor motor = DCMotor.getKrakenX60(1);
    private final DCMotorSim motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, .002, gearRatio),
            motor,
            0.004,
            0.0
    );
    private double appliedVolts;

    public ClimberIOSim() {
        motorSim.setState(Units.degreesToRadians(120), 0);
    }

    @Override
    public void updateInputs(ClimberIO.ClimberIOInputs inputs) {
        motorSim.setInputVoltage(appliedVolts);

        motorSim.update(0.02);

        inputs.connected = true;
        inputs.positionRad = motorSim.getAngularPositionRad();
        inputs.velocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = Math.abs(motorSim.getCurrentDrawAmps());

        inputs.absoluteEncoderConnected = true;
        inputs.absolutePositionRad = motorSim.getAngularPositionRad();
    }

    @Override
    public void setBrakeMode(boolean enable) {
        System.out.println("Setting climber brake mode to " + enable);
    }

    @Override
    public void setVoltage(double voltage) {
        appliedVolts = voltage;
    }
}
