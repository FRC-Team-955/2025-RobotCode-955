package frc.robot.subsystems.intakePivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.PIDF;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.intakePivot.IntakePivotConstants.*;


public class IntakePivotIOSim extends IntakePivotIO {

    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            intakePivotConfig.motorGearRatio(),
            0.1456967969 + Units.lbsToKilograms(15.522) * Math.pow(Units.inchesToMeters(17.0502529), 2),
            Units.inchesToMeters(17.0502529),
            0.12833586,
            1.353,
            true,
            1.353,
            0.00001, // position
            0.00001 // velocity, probably
    );

//    private final PIDF.Profiled pid = PIDF.ofPIDSVAG(
//            intakeConfig.gains().kP(),
//                    intakeConfig.gains().kI(),
//                    intakeConfig.gains().kD(),0.0,0.0,0.0,0.0
//            ).
//            profiled(intakeMaxVelocityRadPerSec,intakeMaxAccelerationRadPerSecSquared);
//            toPID(
//    );

    private final ProfiledPIDController pid = new ProfiledPIDController(
            intakePivotConfig.gains().kP(),
            intakePivotConfig.gains().kI(),
            intakePivotConfig.gains().kD(), new TrapezoidProfile.Constraints(
            intakeMaxVelocityRadPerSec,
            intakeMaxAccelerationRadPerSecSquared
    ));

    private ArmFeedforward ff = intakePivotConfig.gains().toArmFF();
    private boolean closedLoop = true;
    private double appliedVolts;

    public IntakePivotIOSim() {
    }

    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {
        if (closedLoop) {
            appliedVolts = pid.calculate(armSim.getAngleRads())
                    + ff.calculate(armSim.getAngleRads(), pid.getSetpoint().velocity);
            Logger.recordOutput("IntakePivot/SetpointVelocityRadPerSec", pid.getSetpoint().velocity);
        } else {
            pid.reset(new TrapezoidProfile.State(armSim.getAngleRads(), armSim.getVelocityRadPerSec()));
        }
        appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

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
        System.out.println("Setting intakePivot gains");
        pid.setPID(newGains.kP(), newGains.kI(), newGains.kD());
        ff = newGains.toArmFF();
    }
}



