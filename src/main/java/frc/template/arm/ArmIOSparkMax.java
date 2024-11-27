package frc.template.arm;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.*;
import edu.wpi.first.math.util.Units;

public class ArmIOSparkMax extends ArmIO {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkPIDController pid;

    private double gearRatio = 1.0;

    public ArmIOSparkMax(int canID, boolean inverted) {
        motor = new CANSparkMax(canID, CANSparkLowLevel.MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motor.setCANTimeout(250);
        motor.enableVoltageCompensation(12.0);
        motor.setSmartCurrentLimit(40);
        motor.setInverted(inverted);
        motor.burnFlash();

        encoder = motor.getEncoder();

        pid = motor.getPIDController();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.positionRad = Units.rotationsToRadians(encoder.getPosition()) / gearRatio;
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / gearRatio;
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setSetpoint(double setpointPositionRad, double ffVolts) {
        pid.setReference(
                Units.radiansToRotations(setpointPositionRad * gearRatio),
                CANSparkBase.ControlType.kPosition,
                0,
                ffVolts,
                SparkPIDController.ArbFFUnits.kVoltage
        );
    }


    @Override
    public void setPosition(double currentPositionRad) {
        encoder.setPosition(Units.radiansToRotations(currentPositionRad * gearRatio));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        motor.setIdleMode(enabled ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }

    @Override
    public void configurePID(PIDConstants pidConstants) {
        pid.setP(pidConstants.kP);
        pid.setI(pidConstants.kI);
        pid.setD(pidConstants.kD);
        pid.setIZone(pidConstants.iZone);
        pid.setFF(0);
        motor.burnFlash();
    }

    @Override
    public void setGearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
    }
}
