package frc.robot.subsystems.drive.old;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.temp.DriveConstants;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim extends ModuleIO {
    private final DCMotorSim driveSim = new DCMotorSim(DCMotor.getNEO(1), DriveConstants.moduleConfig.driveGearRatio(), 0.025);
    private final DCMotorSim turnSim = new DCMotorSim(DCMotor.getNEO(1), DriveConstants.moduleConfig.turnGearRatio(), 0.004);

    private final double turnAbsoluteInitPosition = Math.random() * 2.0 * Math.PI;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        driveSim.update(0.02);
        turnSim.update(0.02);

        inputs.drivePositionRad = driveSim.getAngularPositionRad();
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        inputs.turnAbsolutePositionRad = turnSim.getAngularPositionRad() + turnAbsoluteInitPosition;
        inputs.turnPositionRad = turnSim.getAngularPositionRad();
        inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveAppliedVolts = MathUtil.clamp(output, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnAppliedVolts = MathUtil.clamp(output, -12.0, 12.0);
        turnSim.setInputVoltage(turnAppliedVolts);
    }
}
