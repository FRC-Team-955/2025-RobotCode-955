package frc.robot.subsystems.intake;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static edu.wpi.first.units.Units.Radians;

public class IntakeIOSim extends IntakeIO {
    private final SingleJointedArmSim pivotSim;
    private PIDController pivotPid;

    private double pivotAppliedVolts;
    private boolean pivotClosedLoop = true;
    private double pivotFfVolts;

    private final FlywheelSim feedSim;
    private PIDController feedPid;

    private double feedAppliedVolts;
    private boolean feedClosedLoop = true;
    private double feedFfVolts;

    /**
     * @param pivot_jKgMetersSquared This can be calculated in Onshape with the "Display mass properties" button in the bottom left. If unsure, you can use 0.1 temporarily.
     */
    public IntakeIOSim(DCMotor pivotMotor, double armLengthMeters, double pivot_jKgMetersSquared, DCMotor feedMotor) {
        pivotSim = new SingleJointedArmSim(
                pivotMotor,
                Intake.PIVOT_GEAR_RATIO,
                pivot_jKgMetersSquared,
                armLengthMeters,
                -Math.PI * 2,
                Math.PI * 2,
                true,
                Intake.PIVOT_INITIAL_POSITION.in(Radians)
        );

        feedSim = new FlywheelSim(feedMotor, Intake.FEED_GEAR_RATIO, 0.004);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        if (pivotClosedLoop) {
            pivotAppliedVolts = pivotPid.calculate(pivotSim.getAngleRads()) + pivotFfVolts;
            pivotSim.setInputVoltage(pivotAppliedVolts);
        }

        pivotSim.update(0.02);

        inputs.pivotPositionRad = pivotSim.getAngleRads();
        inputs.pivotVelocityRadPerSec = pivotSim.getVelocityRadPerSec();
        inputs.pivotAppliedVolts = pivotAppliedVolts;
        inputs.pivotCurrentAmps = Math.abs(pivotSim.getCurrentDrawAmps());

        if (feedClosedLoop) {
            feedAppliedVolts = feedPid.calculate(feedSim.getAngularVelocityRadPerSec()) + feedFfVolts;
            feedSim.setInputVoltage(feedAppliedVolts);
        }

        feedSim.update(0.02);

        inputs.feedPositionRad = 0.0;
        inputs.feedVelocityRadPerSec = feedSim.getAngularVelocityRadPerSec();
        inputs.feedAppliedVolts = feedAppliedVolts;
        inputs.feedCurrentAmps = Math.abs(feedSim.getCurrentDrawAmps());
    }

    @Override
    public void pivotSetVoltage(double volts) {
        pivotAppliedVolts = volts;
        pivotSim.setInputVoltage(pivotAppliedVolts);
        pivotClosedLoop = false;
    }

    @Override
    public void pivotSetSetpoint(double setpointPositionRad, double ffVolts) {
        this.pivotFfVolts = ffVolts;
        pivotPid.setSetpoint(setpointPositionRad);
        pivotClosedLoop = true;
    }

    @Override
    public void pivotSetPosition(double currentPositionRad) {
        pivotSim.setState(currentPositionRad, 0);
    }

    @Override
    public void pivotConfigurePID(PIDConstants pidConstants) {
        pivotPid = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD);
        pivotPid.setIZone(pidConstants.iZone);
    }

    @Override
    public void feedSetVoltage(double volts) {
        feedAppliedVolts = volts;
        feedSim.setInputVoltage(feedAppliedVolts);
        feedClosedLoop = false;
    }

    @Override
    public void feedSetSetpoint(double setpointVelocityRadPerSec, double ffVolts) {
        feedFfVolts = ffVolts;
        feedPid.setSetpoint(setpointVelocityRadPerSec);
        feedClosedLoop = true;
    }

    @Override
    public void feedConfigurePID(PIDConstants pidConstants) {
        feedPid = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD);
        feedPid.setIZone(pidConstants.iZone);
    }
}
