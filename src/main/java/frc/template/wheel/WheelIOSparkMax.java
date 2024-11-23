package frc.template.wheel;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.*;
import edu.wpi.first.math.util.Units;
import frc.robot.util.MotorFlags;

import java.util.EnumSet;

public class WheelIOSparkMax extends WheelIO {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkPIDController pid;

    private double gearRatio = 1.0;

    public WheelIOSparkMax(int canID, EnumSet<MotorFlags> flags) {
        motor = new CANSparkMax(canID, CANSparkLowLevel.MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(flags.contains(MotorFlags.IdleModeBrake) ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
        motor.setCANTimeout(250);
        motor.enableVoltageCompensation(12.0);
        motor.setSmartCurrentLimit(40);
        motor.setInverted(flags.contains(MotorFlags.Inverted));
        motor.burnFlash();

        encoder = motor.getEncoder();

        pid = motor.getPIDController();
    }

    @Override
    public void updateInputs(WheelIOInputs inputs) {
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
    public void setSetpoint(double setpointVelocityRadPerSec, double ffVolts) {
        pid.setReference(
                Units.radiansPerSecondToRotationsPerMinute(setpointVelocityRadPerSec * gearRatio),
                CANSparkBase.ControlType.kVelocity,
                0,
                ffVolts,
                SparkPIDController.ArbFFUnits.kVoltage
        );
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
