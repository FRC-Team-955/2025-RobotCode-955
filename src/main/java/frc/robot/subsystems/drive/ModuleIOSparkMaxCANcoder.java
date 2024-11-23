package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and CANcoder
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Inputs/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMaxCANcoder extends ModuleIO {
    // Gear ratios for SDS MK4i L2, adjust as necessary
    public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

    private final CANSparkMax driveSparkMax;
    private final CANSparkMax turnSparkMax;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnRelativeEncoder;
    private final CANcoder cancoder;
    private final StatusSignal<Double> turnAbsolutePosition;

    private final boolean isTurnMotorInverted = true;
    private final double absoluteEncoderOffsetRad;

    public ModuleIOSparkMaxCANcoder(int index) {
        // See class level documentation comment for info on calibrating encoder offsets
        switch (index) {
            case 0 -> { // FL
                driveSparkMax = new CANSparkMax(2, MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(1, MotorType.kBrushless);
                cancoder = new CANcoder(18);
                absoluteEncoderOffsetRad = 0.192;
            }
            case 1 -> { // FR
                driveSparkMax = new CANSparkMax(14, MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(15, MotorType.kBrushless);
                cancoder = new CANcoder(19);
                absoluteEncoderOffsetRad = -1.411;
            }
            case 2 -> { // BL
                driveSparkMax = new CANSparkMax(5, MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(6, MotorType.kBrushless);
                cancoder = new CANcoder(20);
                absoluteEncoderOffsetRad = -0.008;
            }
            case 3 -> { // BR
                driveSparkMax = new CANSparkMax(11, MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(12, MotorType.kBrushless);
                cancoder = new CANcoder(21);
                absoluteEncoderOffsetRad = -2.832;
            }
            default -> throw new RuntimeException("unreachable");
        }

        driveSparkMax.restoreFactoryDefaults();
        turnSparkMax.restoreFactoryDefaults();

        driveSparkMax.setCANTimeout(250);
        turnSparkMax.setCANTimeout(250);

        driveEncoder = driveSparkMax.getEncoder();
        turnRelativeEncoder = turnSparkMax.getEncoder();

        turnSparkMax.setInverted(isTurnMotorInverted);
        driveSparkMax.setSmartCurrentLimit(40);
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
        inputs.drivePositionRad = Units.rotationsToRadians(driveEncoder.getPosition()) / DRIVE_GEAR_RATIO;
        inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / DRIVE_GEAR_RATIO;
        inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
        inputs.driveCurrentAmps = driveSparkMax.getOutputCurrent();

        inputs.turnAbsolutePositionRad = Units.rotationsToRadians(turnAbsolutePosition.getValueAsDouble()) - absoluteEncoderOffsetRad;
        inputs.turnPositionRad = Units.rotationsToRadians(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
        inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity()) / TURN_GEAR_RATIO;
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
