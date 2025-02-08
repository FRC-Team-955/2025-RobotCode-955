package frc.robot.subsystems.coralintake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.PIDF;

public class PivotIOSim extends PivotIO {
    private final SingleJointedArmSim motorSim;
    private final PIDController pid;
    private final ArmFeedforward ff;

    private boolean closedLoop = true;
    private double appliedVolts;

    public PivotIOSim(PIDF pidf) {
        // If we need to test different geometries, we can put this in coralintakeconstants
        // TODO: Figure out
        motorSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                CoralIntakeConstants.pivotConfig.motorGearRatio(),
                Units.lbsToKilograms(Units.inchesToMeters(Units.inchesToMeters(429.942))),
                Units.inchesToMeters(20),
                0.12833586,
                1.353,
                true,
                1.353,
                0.002, 0.002
        );
        pid = pidf.toPID();
        ff = pidf.toArmFF();
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
