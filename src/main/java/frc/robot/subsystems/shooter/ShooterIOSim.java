package frc.robot.subsystems.shooter;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static edu.wpi.first.units.Units.Radians;

public class ShooterIOSim extends ShooterIO {
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

    private final FlywheelSim flywheelTopSim;
    private PIDController flywheelTopPid;
    private double flywheelTopAppliedVolts;
    private boolean flywheelTopClosedLoop = true;
    private double flywheelTopFfVolts;

    private final FlywheelSim flywheelBottomSim;
    private PIDController flywheelBottomPid;
    private double flywheelBottomAppliedVolts;
    private boolean flywheelBottomClosedLoop = true;
    private double flywheelBottomFfVolts;

    /**
     * @param pivot_jKgMetersSquared This can be calculated in Onshape with the "Display mass properties" button in the bottom left. If unsure, you can use 0.1 temporarily.
     */
    public ShooterIOSim(DCMotor pivotMotor, double armLengthMeters, double pivot_jKgMetersSquared, DCMotor feedMotor, DCMotor flywheelTopMotor, DCMotor flywheelBottomMotor) {
        pivotSim = new SingleJointedArmSim(
                pivotMotor,
                Shooter.PIVOT_GEAR_RATIO,
                pivot_jKgMetersSquared,
                armLengthMeters,
                -Math.PI * 2,
                Math.PI * 2,
                true,
                Shooter.PIVOT_INITIAL_POSITION.in(Radians)
        );

        feedSim = new FlywheelSim(feedMotor, Shooter.FEED_GEAR_RATIO, 0.004);

        flywheelTopSim = new FlywheelSim(flywheelTopMotor, Shooter.FLYWHEEL_GEAR_RATIO, 0.004);

        flywheelBottomSim = new FlywheelSim(flywheelBottomMotor, Shooter.FLYWHEEL_GEAR_RATIO, 0.004);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.hasNote = DriverStation.isAutonomous()
                ? Shooter.get().getGoal() == Shooter.Goal.HANDOFF_FEED
                : Timer.getFPGATimestamp() % 1 > 0.5;

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

        if (feedClosedLoop) {
            feedAppliedVolts = feedPid.calculate(feedSim.getAngularVelocityRadPerSec()) + feedFfVolts;
            feedSim.setInputVoltage(feedAppliedVolts);
        }

        feedSim.update(0.02);

        inputs.feedPositionRad = 0.0;
        inputs.feedVelocityRadPerSec = feedSim.getAngularVelocityRadPerSec();
        inputs.feedAppliedVolts = feedAppliedVolts;
        inputs.feedCurrentAmps = Math.abs(feedSim.getCurrentDrawAmps());

        if (flywheelTopClosedLoop) {
            flywheelTopAppliedVolts = flywheelTopPid.calculate(flywheelTopSim.getAngularVelocityRadPerSec()) + flywheelTopFfVolts;
            flywheelTopSim.setInputVoltage(flywheelTopAppliedVolts);
        }

        flywheelTopSim.update(0.02);

        inputs.flywheelTopPositionRad = 0.0;
        inputs.flywheelTopVelocityRadPerSec = flywheelTopSim.getAngularVelocityRadPerSec();
        inputs.flywheelTopAppliedVolts = flywheelTopAppliedVolts;
        inputs.flywheelTopCurrentAmps = Math.abs(flywheelTopSim.getCurrentDrawAmps());

        if (flywheelBottomClosedLoop) {
            flywheelBottomAppliedVolts = flywheelBottomPid.calculate(flywheelBottomSim.getAngularVelocityRadPerSec()) + flywheelBottomFfVolts;
            flywheelBottomSim.setInputVoltage(flywheelBottomAppliedVolts);
        }

        flywheelBottomSim.update(0.02);

        inputs.flywheelBottomPositionRad = 0.0;
        inputs.flywheelBottomVelocityRadPerSec = flywheelBottomSim.getAngularVelocityRadPerSec();
        inputs.flywheelBottomAppliedVolts = flywheelBottomAppliedVolts;
        inputs.flywheelBottomCurrentAmps = Math.abs(flywheelBottomSim.getCurrentDrawAmps());
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

    @Override
    public void flywheelsSetVoltage(double volts) {
        flywheelBottomAppliedVolts = volts;
        flywheelBottomSim.setInputVoltage(flywheelBottomAppliedVolts);
        flywheelBottomClosedLoop = false;

        flywheelTopAppliedVolts = volts;
        flywheelTopSim.setInputVoltage(flywheelTopAppliedVolts);
        flywheelTopClosedLoop = false;
    }

    @Override
    public void flywheelsSetSetpoint(double setpointVelocityRadPerSec, double ffTopVolts, double ffBottomVolts) {
        flywheelTopFfVolts = ffTopVolts;
        flywheelTopPid.setSetpoint(setpointVelocityRadPerSec);
        flywheelTopClosedLoop = true;

        flywheelBottomFfVolts = ffBottomVolts;
        flywheelBottomPid.setSetpoint(setpointVelocityRadPerSec);
        flywheelBottomClosedLoop = true;
    }

    @Override
    public void flywheelsTopConfigurePID(PIDConstants pidConstants) {
        flywheelTopPid = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD);
        flywheelTopPid.setIZone(pidConstants.iZone);
    }

    @Override
    public void flywheelsBottomConfigurePID(PIDConstants pidConstants) {
        flywheelBottomPid = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD);
        flywheelBottomPid.setIZone(pidConstants.iZone);
    }
}
