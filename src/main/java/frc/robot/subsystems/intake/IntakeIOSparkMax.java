package frc.robot.subsystems.intake;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.*;
import edu.wpi.first.math.util.Units;

public class IntakeIOSparkMax extends IntakeIO {
    private final CANSparkMax pivotMotor;
    private final RelativeEncoder pivotEncoder;
    private final SparkPIDController pivotPid;

    private final CANSparkMax feedMotor;
    private final RelativeEncoder feedEncoder;
    private final SparkPIDController feedPid;

    public IntakeIOSparkMax(int pivotCanID, int feedCanID) {
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
        feedMotor.setInverted(true);
        feedMotor.burnFlash();
        feedEncoder = feedMotor.getEncoder();
        feedPid = feedMotor.getPIDController();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.pivotPositionRad = Units.rotationsToRadians(pivotEncoder.getPosition()) / Intake.PIVOT_GEAR_RATIO;
        inputs.pivotVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(pivotEncoder.getVelocity()) / Intake.PIVOT_GEAR_RATIO;
        inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        inputs.pivotCurrentAmps = pivotMotor.getOutputCurrent();

        inputs.feedPositionRad = Units.rotationsToRadians(feedEncoder.getPosition()) / Intake.FEED_GEAR_RATIO;
        inputs.feedVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(feedEncoder.getVelocity()) / Intake.FEED_GEAR_RATIO;
        inputs.feedAppliedVolts = feedMotor.getAppliedOutput() * feedMotor.getBusVoltage();
        inputs.feedCurrentAmps = feedMotor.getOutputCurrent();
    }

    @Override
    public void pivotSetVoltage(double volts) {
        pivotMotor.setVoltage(volts);
    }

    @Override
    public void pivotSetSetpoint(double setpointPositionRad, double ffVolts) {
        pivotPid.setReference(
                Units.radiansToRotations(setpointPositionRad * Intake.PIVOT_GEAR_RATIO),
                CANSparkBase.ControlType.kPosition,
                0,
                ffVolts,
                SparkPIDController.ArbFFUnits.kVoltage
        );
    }


    @Override
    public void pivotSetPosition(double currentPositionRad) {
        pivotEncoder.setPosition(Units.radiansToRotations(currentPositionRad * Intake.PIVOT_GEAR_RATIO));
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
                Units.radiansPerSecondToRotationsPerMinute(setpointVelocityRadPerSec * Intake.FEED_GEAR_RATIO),
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
}
