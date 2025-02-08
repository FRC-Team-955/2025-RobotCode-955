package frc.robot.subsystems.coralintake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim extends PivotIO {
    private final SingleJointedArmSim motorSim;
    private final PIDController pid;
    private final ArmFeedforward ff;

    private boolean closedLoop = true;
    private double appliedVolts;

    public PivotIOSim(DCMotor motor, ArmFeedforward ff) {
        // If we need to test different geometries, we can put this in coralintakeconstants
        motorSim = new SingleJointedArmSim(
            motor,
            CoralIntakeConstants.pivotConfigSim.motorGearRatio(),
            0,
            0,
            0,
            0,
            true,
            0,
            0.004
        );
        pid = CoralIntakeConstants.pivotConfigSim.motorGains().toPID();
        this.ff = ff;
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        if (closedLoop) {
            appliedVolts = pid.calculate(motorSim.getAngleRads()) + ff.calculate(motorSim.getAngleRads(), motorSim.getVelocityRadPerSec());
        }

        motorSim.setInputVoltage(appliedVolts);

        motorSim.update(0.02);

        inputs.connected = true;
        inputs.positionRad = motorSim.getAngleRads();
        inputs.velocityRadPerSec = motorSim.getVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = Math.abs(motorSim.getCurrentDrawAmps());
    }

    @Override
    public void setOpenLoop(double output) {
        closedLoop = false;
        appliedVolts = output;
    }

    @Override
    public void setClosedLoop(double positionRad) {
        closedLoop = true;
        pid.setSetpoint(positionRad);
    }
}
