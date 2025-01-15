package frc.robot.subsystems.drive.old;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.temp.DriveConstants;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO or NEO 550), and CANcoder
 */
public class ModuleIOSparkMaxCANcoder extends ModuleIO {
    private final CANSparkMax driveSparkMax;
    private final CANSparkMax turnSparkMax;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnRelativeEncoder;
    private final CANcoder cancoder;
    private final StatusSignal<Double> turnAbsolutePosition;

    private final boolean isTurnMotorInverted = true;
    private final double absoluteEncoderOffsetRad;

    public ModuleIOSparkMaxCANcoder(
            int driveCanID,
            int turnCanID,
            int cancoderCanID,
            double absoluteEncoderOffsetRad
    ) {
        driveSparkMax = new CANSparkMax(driveCanID, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(turnCanID, MotorType.kBrushless);
        cancoder = new CANcoder(cancoderCanID);
        this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;

        driveSparkMax.restoreFactoryDefaults();
        turnSparkMax.restoreFactoryDefaults();

        driveSparkMax.setCANTimeout(250);
        turnSparkMax.setCANTimeout(250);

        driveEncoder = driveSparkMax.getEncoder();
        turnRelativeEncoder = turnSparkMax.getEncoder();

        turnSparkMax.setInverted(isTurnMotorInverted);
        driveSparkMax.setSmartCurrentLimit(60);
        turnSparkMax.setSmartCurrentLimit(30);
        driveSparkMax.enableVoltageCompensation(12.0);
        turnSparkMax.enableVoltageCompensation(12.0);

        driveEncoder.setPosition(0.0);
        driveEncoder.setMeasurementPeriod(10);
        driveEncoder.setAverageDepth(2);

        turnRelativeEncoder.setPosition(0.0);
        turnRelativeEncoder.setMeasurementPeriod(10);
        turnRelativeEncoder.setAverageDepth(2);

        driveSparkMax.setCANTimeout(0);
        turnSparkMax.setCANTimeout(0);

        driveSparkMax.burnFlash();
        turnSparkMax.burnFlash();

        cancoder.getConfigurator().apply(new CANcoderConfiguration());
        turnAbsolutePosition = cancoder.getAbsolutePosition();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionRad = Units.rotationsToRadians(driveEncoder.getPosition()) / DriveConstants.moduleConfig.driveGearRatio();
        inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / DriveConstants.moduleConfig.driveGearRatio();
        inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
        inputs.driveCurrentAmps = driveSparkMax.getOutputCurrent();

        inputs.turnAbsolutePositionRad = Units.rotationsToRadians(turnAbsolutePosition.getValueAsDouble()) - absoluteEncoderOffsetRad;
        inputs.turnPositionRad = Units.rotationsToRadians(turnRelativeEncoder.getPosition() / DriveConstants.moduleConfig.turnGearRatio());
        inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity()) / DriveConstants.moduleConfig.turnGearRatio();
        inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
        inputs.turnCurrentAmps = turnSparkMax.getOutputCurrent();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveSparkMax.setVoltage(volts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnSparkMax.setVoltage(volts);
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }
}
