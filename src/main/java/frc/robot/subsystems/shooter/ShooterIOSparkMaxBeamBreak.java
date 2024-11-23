package frc.robot.subsystems.shooter;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterIOSparkMaxBeamBreak extends ShooterIO {
    private final DigitalInput beamBreak;

    private final CANSparkMax pivotMotor;
    private final RelativeEncoder pivotEncoder;
    private final SparkPIDController pivotPid;

    private final CANSparkMax feedMotor;
    private final RelativeEncoder feedEncoder;
    private final SparkPIDController feedPid;

    private final CANSparkMax flywheelTopMotor;
    private final RelativeEncoder flywheelTopEncoder;
    private final SparkPIDController flywheelTopPid;

    private final CANSparkMax flywheelBottomMotor;
    private final RelativeEncoder flywheelBottomEncoder;
    private final SparkPIDController flywheelBottomPid;

    public ShooterIOSparkMaxBeamBreak(int beamBreakPwmID, int pivotCanID, int feedCanID, int flywheelTopCanID, int flywheelBottomCanID) {
        beamBreak = new DigitalInput(beamBreakPwmID);

        pivotMotor = new CANSparkMax(pivotCanID, CANSparkLowLevel.MotorType.kBrushless);
        pivotMotor.restoreFactoryDefaults();
        pivotMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        pivotMotor.setCANTimeout(250);
        pivotMotor.enableVoltageCompensation(12.0);
        pivotMotor.setSmartCurrentLimit(40);
        pivotMotor.burnFlash();
        pivotEncoder = pivotMotor.getEncoder();
        pivotPid = pivotMotor.getPIDController();

        feedMotor = new CANSparkMax(feedCanID, CANSparkLowLevel.MotorType.kBrushless);
        feedMotor.restoreFactoryDefaults();
        feedMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        feedMotor.setCANTimeout(250);
        feedMotor.enableVoltageCompensation(12.0);
        feedMotor.setSmartCurrentLimit(40);
        feedMotor.setInverted(false);
        feedMotor.burnFlash();
        feedEncoder = feedMotor.getEncoder();
        feedPid = feedMotor.getPIDController();

        flywheelTopMotor = new CANSparkMax(flywheelTopCanID, CANSparkLowLevel.MotorType.kBrushless);
        flywheelTopMotor.restoreFactoryDefaults();
        flywheelTopMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        flywheelTopMotor.setCANTimeout(250);
        flywheelTopMotor.enableVoltageCompensation(12.0);
        flywheelTopMotor.setSmartCurrentLimit(40);
        flywheelTopMotor.setInverted(true);
        flywheelTopMotor.burnFlash();
        flywheelTopEncoder = flywheelTopMotor.getEncoder();
        flywheelTopPid = flywheelTopMotor.getPIDController();

        flywheelBottomMotor = new CANSparkMax(flywheelBottomCanID, CANSparkLowLevel.MotorType.kBrushless);
        flywheelBottomMotor.restoreFactoryDefaults();
        flywheelBottomMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        flywheelBottomMotor.setCANTimeout(250);
        flywheelBottomMotor.enableVoltageCompensation(12.0);
        flywheelBottomMotor.setSmartCurrentLimit(40);
        flywheelBottomMotor.setInverted(true);
        flywheelBottomMotor.burnFlash();
        flywheelBottomEncoder = flywheelBottomMotor.getEncoder();
        flywheelBottomPid = flywheelBottomMotor.getPIDController();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.hasNote = !beamBreak.get();

        inputs.pivotPositionRad = Units.rotationsToRadians(pivotEncoder.getPosition()) / Shooter.PIVOT_GEAR_RATIO;
        inputs.pivotVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(pivotEncoder.getVelocity()) / Shooter.PIVOT_GEAR_RATIO;
        inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        inputs.pivotCurrentAmps = pivotMotor.getOutputCurrent();

        inputs.feedPositionRad = Units.rotationsToRadians(feedEncoder.getPosition()) / Shooter.FEED_GEAR_RATIO;
        inputs.feedVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(feedEncoder.getVelocity()) / Shooter.FEED_GEAR_RATIO;
        inputs.feedAppliedVolts = feedMotor.getAppliedOutput() * feedMotor.getBusVoltage();
        inputs.feedCurrentAmps = feedMotor.getOutputCurrent();

        inputs.flywheelTopPositionRad = Units.rotationsToRadians(flywheelTopEncoder.getPosition()) / Shooter.FLYWHEEL_GEAR_RATIO;
        inputs.flywheelTopVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(flywheelTopEncoder.getVelocity()) / Shooter.FLYWHEEL_GEAR_RATIO;
        inputs.flywheelTopAppliedVolts = flywheelTopMotor.getAppliedOutput() * flywheelTopMotor.getBusVoltage();
        inputs.flywheelTopCurrentAmps = flywheelTopMotor.getOutputCurrent();

        inputs.flywheelBottomPositionRad = Units.rotationsToRadians(flywheelBottomEncoder.getPosition()) / Shooter.FLYWHEEL_GEAR_RATIO;
        inputs.flywheelBottomVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(flywheelBottomEncoder.getVelocity()) / Shooter.FLYWHEEL_GEAR_RATIO;
        inputs.flywheelBottomAppliedVolts = flywheelBottomMotor.getAppliedOutput() * flywheelBottomMotor.getBusVoltage();
        inputs.flywheelBottomCurrentAmps = flywheelBottomMotor.getOutputCurrent();
    }

    @Override
    public void pivotSetVoltage(double volts) {
        pivotMotor.setVoltage(volts);
    }

    @Override
    public void pivotSetSetpoint(double setpointPositionRad, double ffVolts) {
        pivotPid.setReference(
                Units.radiansToRotations(setpointPositionRad * Shooter.PIVOT_GEAR_RATIO),
                CANSparkBase.ControlType.kPosition,
                0,
                ffVolts,
                SparkPIDController.ArbFFUnits.kVoltage
        );
    }


    @Override
    public void pivotSetPosition(double currentPositionRad) {
        pivotEncoder.setPosition(Units.radiansToRotations(currentPositionRad * Shooter.PIVOT_GEAR_RATIO));
    }

    @Override
    public void pivotSetBrakeMode(boolean enabled) {
        pivotMotor.setIdleMode(enabled ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }

    @Override
    public void pivotConfigurePID(PIDConstants pidConstants) {
        pivotPid.setP(pidConstants.kP);
        pivotPid.setI(pidConstants.kI);
        pivotPid.setD(pidConstants.kD);
        pivotPid.setIZone(pidConstants.iZone);
        pivotPid.setFF(0);
        pivotMotor.burnFlash();
    }

    @Override
    public void feedSetVoltage(double volts) {
        feedMotor.setVoltage(volts);
    }

    @Override
    public void feedSetSetpoint(double setpointVelocityRadPerSec, double ffVolts) {
        feedPid.setReference(
                Units.radiansPerSecondToRotationsPerMinute(setpointVelocityRadPerSec * Shooter.FEED_GEAR_RATIO),
                CANSparkBase.ControlType.kVelocity,
                0,
                ffVolts,
                SparkPIDController.ArbFFUnits.kVoltage
        );
    }

    @Override
    public void feedSetBrakeMode(boolean enabled) {
        feedMotor.setIdleMode(enabled ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }

    @Override
    public void feedConfigurePID(PIDConstants pidConstants) {
        feedPid.setP(pidConstants.kP);
        feedPid.setI(pidConstants.kI);
        feedPid.setD(pidConstants.kD);
        feedPid.setIZone(pidConstants.iZone);
        feedPid.setFF(0);
        feedMotor.burnFlash();
    }

    @Override
    public void flywheelsSetVoltage(double volts) {
        flywheelTopMotor.setVoltage(volts);
        flywheelBottomMotor.setVoltage(volts);
    }

    @Override
    public void flywheelsSetSetpoint(double setpointVelocityRadPerSec, double ffTopVolts, double ffBottomVolts) {
        flywheelTopPid.setReference(
                Units.radiansPerSecondToRotationsPerMinute(setpointVelocityRadPerSec * Shooter.FLYWHEEL_GEAR_RATIO),
                CANSparkBase.ControlType.kVelocity,
                0,
                ffTopVolts,
                SparkPIDController.ArbFFUnits.kVoltage
        );

        flywheelBottomPid.setReference(
                Units.radiansPerSecondToRotationsPerMinute(setpointVelocityRadPerSec * Shooter.FLYWHEEL_GEAR_RATIO),
                CANSparkBase.ControlType.kVelocity,
                0,
                ffBottomVolts,
                SparkPIDController.ArbFFUnits.kVoltage
        );
    }

    @Override
    public void flywheelsSetBrakeMode(boolean enabled) {
        flywheelTopMotor.setIdleMode(enabled ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
        flywheelBottomMotor.setIdleMode(enabled ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }

    @Override
    public void flywheelsTopConfigurePID(PIDConstants pidConstants
    ) {
        flywheelTopPid.setP(pidConstants.kP);
        flywheelTopPid.setI(pidConstants.kI);
        flywheelTopPid.setD(pidConstants.kD);
        flywheelTopPid.setIZone(pidConstants.iZone);
        flywheelTopPid.setFF(0);
        flywheelTopMotor.burnFlash();
    }

    @Override
    public void flywheelsBottomConfigurePID(PIDConstants pidConstants) {
        flywheelBottomPid.setP(pidConstants.kP);
        flywheelBottomPid.setI(pidConstants.kI);
        flywheelBottomPid.setD(pidConstants.kD);
        flywheelBottomPid.setIZone(pidConstants.iZone);
        flywheelBottomPid.setFF(0);
        flywheelBottomMotor.burnFlash();
    }
}
