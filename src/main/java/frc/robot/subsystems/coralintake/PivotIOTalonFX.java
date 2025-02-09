package frc.robot.subsystems.coralintake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import static frc.robot.subsystems.coralintake.CoralIntakeConstants.pivotConfig;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class PivotIOTalonFX extends PivotIO {
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final SwerveModuleConstants.ClosedLoopOutputType driveClosedLoopOutput = SwerveModuleConstants.ClosedLoopOutputType.Voltage;

    // Hardware objects
    private final TalonFX driveTalon;
    private final DutyCycleEncoder encoder;

    private final TalonFXConfiguration motorConfig;

    // Voltage control requests
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);

    // Torque-current control requests
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
            new PositionTorqueCurrentFOC(0.0);

    // Inputs from drive motor
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveCurrent;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

    public PivotIOTalonFX(
            int driveCanID,
            int encoderID,
            double absoluteEncoderOffsetRad
    ) {
        driveTalon = new TalonFX(driveCanID);
        encoder = new DutyCycleEncoder(encoderID, 2 * Math.PI, absoluteEncoderOffsetRad);

        // Configure pivot motor
        motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Slot0 = Slot0Configs.from(pivotConfig.gains().toPhoenix());
        motorConfig.Feedback.SensorToMechanismRatio = pivotConfig.motorGearRatio();
        motorConfig.Feedback.FeedbackRotorOffset = Units.radiansToRotations(encoder.get());
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = pivotConfig.currentLimit();
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = pivotConfig.currentLimit();
        motorConfig.CurrentLimits.StatorCurrentLimit = pivotConfig.currentLimit();
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0;
        motorConfig.MotionMagic.MotionMagicAcceleration = 1000.0;
        motorConfig.MotionMagic.MotionMagicExpo_kV = 0.12;
        motorConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
        motorConfig.ClosedLoopGeneral.ContinuousWrap = false;
        motorConfig.MotorOutput.Inverted =
                pivotConfig.motorInverted()
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> driveTalon.getConfigurator().apply(motorConfig, 0.25));

        // Configure encoder
        encoder.setInverted(pivotConfig.encoderInverted());

        // Create drive status signals
        drivePosition = driveTalon.getPosition();
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getStatorCurrent();

        ParentDevice.optimizeBusUtilizationForAll(driveTalon);
    }

    @Override
    public void updateInputs(PivotIO.PivotIOInputs inputs) {
        // Refresh all signals
        var driveStatus = BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);

        // Update drive inputs
        inputs.connected = driveConnectedDebounce.calculate(driveStatus.isOK());
        inputs.positionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.velocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.appliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.currentAmps = driveCurrent.getValueAsDouble();
        inputs.absoluteEncoderConnected = turnEncoderConnectedDebounce.calculate(encoder.isConnected());
        inputs.absolutePositionRad = encoder.get();
    }

    @Override
    public void setBrakeMode(boolean enable) {
        motorConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        tryUntilOk(5, () -> driveTalon.getConfigurator().apply(motorConfig, 0.25));
    }

    @Override
    public void setOpenLoop(double output) {
        driveTalon.setControl(switch (driveClosedLoopOutput) {
            case Voltage -> voltageRequest.withOutput(output);
            case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
    }

    @Override
    public void setClosedLoop(double positionRad) {
        double positionRot = Units.radiansToRotations(positionRad);
        driveTalon.setControl(switch (driveClosedLoopOutput) {
            case Voltage -> positionVoltageRequest.withPosition(positionRot);
            case TorqueCurrentFOC -> positionTorqueCurrentRequest.withPosition(positionRot);
        });
    }
}
