package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import static frc.robot.Util.phoenixTryUntilOk;

/**
 * Module IO implementation for TalonFX drive motor controller, SparkMax turn motor controller (NEO or NEO 550), and CANcoder
 */
public class ModuleIOTalonFXSparkMaxCANcoder extends ModuleIO {
    private final TalonFX driveTalon;
    private final CANSparkMax turnSparkMax;
    private final CANcoder cancoder;

    private final StatusSignal<Angle> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveCurrent;

    private final RelativeEncoder turnRelativeEncoder;
    private final StatusSignal<Double> turnAbsolutePosition;

    private final boolean isTurnMotorInverted = true;
    private final double absoluteEncoderOffsetRad;

    private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(8);

    private final Queue<Double> timestampQueue;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

    public ModuleIOTalonFXSparkMaxCANcoder(
            int driveCanID,
            int turnCanID,
            int cancoderCanID,
            double absoluteEncoderOffsetRad
    ) {
        driveTalon = new TalonFX(driveCanID);
        turnSparkMax = new CANSparkMax(turnCanID, MotorType.kBrushless);
        cancoder = new CANcoder(cancoderCanID);
        this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;

        turnSparkMax.restoreFactoryDefaults();
        turnSparkMax.setCANTimeout(250);
        turnRelativeEncoder = turnSparkMax.getEncoder();
        turnSparkMax.setInverted(isTurnMotorInverted);
        turnSparkMax.setSmartCurrentLimit(30);
        turnSparkMax.enableVoltageCompensation(12.0);
        turnRelativeEncoder.setPosition(0.0);
        turnRelativeEncoder.setMeasurementPeriod(10);
        turnRelativeEncoder.setAverageDepth(2);
        turnSparkMax.setCANTimeout(0);
        turnSparkMax.burnFlash();

        // Drive Motor Configs
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        phoenixTryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
        phoenixTryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

        drivePosition = driveTalon.getPosition();
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getSupplyCurrent();

        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.MagnetOffset = Units.radiansToRotations(-absoluteEncoderOffsetRad);
        cancoder.getConfigurator().apply(new CANcoderConfiguration());
        turnAbsolutePosition = cancoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, drivePosition); // Required for odometry, use faster rate
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                driveVelocity,
                driveAppliedVolts,
                driveCurrent,
                turnAbsolutePosition
        );
        driveTalon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveAppliedVolts,
                driveCurrent,
                turnAbsolutePosition
        );

        inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DriveConstants.moduleConfig.driveGearRatio();
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DriveConstants.moduleConfig.driveGearRatio();
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

        inputs.turnAbsolutePositionRad = Units.rotationsToRadians(turnAbsolutePosition.getValueAsDouble());
        inputs.turnPositionRad = Units.rotationsToRadians(turnRelativeEncoder.getPosition() / DriveConstants.moduleConfig.turnGearRatio());
        inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity()) / DriveConstants.moduleConfig.turnGearRatio();
        inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
        inputs.turnCurrentAmps = turnSparkMax.getOutputCurrent();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveTalon.setControl(new VoltageOut(volts));
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnSparkMax.setVoltage(volts);
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        brakeModeExecutor.execute(
                () -> {
                    synchronized (driveConfig) {
                        driveConfig.MotorOutput.NeutralMode =
                                enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                        phoenixTryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
                    }
                });
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }
}
