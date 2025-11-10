package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.PIDF;
import org.littletonrobotics.junction.Logger;
import static frc.robot.subsystems.intake.IntakeConstants.*;

public class IntakeIOSim extends IntakeIO {
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            intakeConfig.motorGearRatio(),
            0.1456967969 + Units.lbsToKilograms(15.522) * Math.pow(Units.inchesToMeters(17.0502529), 2),
            Units.inchesToMeters(17.0502529),
            0.12833586,
            1.353,
            true,
            1.353,
            0.00001,
            0.00001
    );
    private final ProfiledPIDController pid = new ProfiledPIDController(
            intakeConfig.gains().kP(),
            intakeConfig.gains().kI(),
            intakeConfig.gains().kD(),
            new TrapezoidProfile.Constraints(
                    intakeMaxVelocityRadPerSec,
                    intakeMaxAccelerationRadPerSecSquared
            )
    );
    private static ArmFeedforward ff = intakeConfig.gains().toArmFF();

    private boolean closedLoop = true;
    private double appliedVolts;

    public IntakeIOSim() {}

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        if (closedLoop) {
            appliedVolts = pid.calculate(armSim.getAngleRads())
                    + ff.calculate(armSim.getAngleRads(), pid.getSetpoint().velocity);
            Logger.recordOutput("Intake/Pivot/SetpointVelocityRadPerSec", pid.getSetpoint().velocity);
        } else {
            pid.reset(new TrapezoidProfile.State(armSim.getAngleRads(), armSim.getVelocityRadPerSec()));
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
    public void setOpenLoop(double output) {
        closedLoop = false;
        appliedVolts = output;
    }

    @Override
    public void setClosedLoop(double positionRad) {
        closedLoop = true;
        pid.setGoal(new TrapezoidProfile.State(positionRad, 0));
    }

    @Override
    public void setIntakePIDF(PIDF newGains) {
        System.out.println("Setting intake gains");
        pid.setP(newGains.kP());
        pid.setI(newGains.kI());
        pid.setD(newGains.kD());
        ff = newGains.toArmFF();
    }
}
