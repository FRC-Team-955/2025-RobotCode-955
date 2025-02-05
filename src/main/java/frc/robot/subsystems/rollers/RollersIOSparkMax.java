package frc.robot.subsystems.rollers;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.drive.DriveConstants;

import java.util.function.DoubleSupplier;

import static frc.robot.util.SparkUtil.*;
import static frc.robot.util.SparkUtil.tryUntilOkAsync;

public class RollersIOSparkMax extends RollersIO {

    // Hardware objects
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkMaxConfig config;

    // Closed loop controllers
    private final SparkClosedLoopController controller;

    // Connection debouncers
    private final Debouncer connectedDebounce = new Debouncer(0.5);

    private final SimpleMotorFeedforward feedforward;

    public RollersIOSparkMax(
            int canID,
            RollersConfig rollersConfig
    ) {
        motor = new SparkMax(canID, SparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();

        feedforward = rollersConfig.gains().toSimpleFF();

        // Configure drive motor
        config = new SparkMaxConfig();
        config
                .inverted(rollersConfig.inverted())
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(rollersConfig.currentLimit())
                .voltageCompensation(12.0);
        config
                .encoder
                .positionConversionFactor(2 * Math.PI / rollersConfig.gearRatio()) // Rotor Rotations -> Wheel Radians
                .velocityConversionFactor((2 * Math.PI) / 60.0 / rollersConfig.gearRatio()) // Rotor RPM -> Wheel Rad/Sec
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        config
                .closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
        rollersConfig.gains().applySpark(config.closedLoop);
        tryUntilOk(5, () -> motor.configure(
                config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
        tryUntilOk(5, () -> encoder.setPosition(0.0));
    }

    @Override
    public void updateInputs(RollersIO.RollersIOInputs inputs) {
        // Update drive inputs
        sparkStickyFault = false;
        ifOk(motor, encoder::getPosition, (value) -> inputs.positionRad = value);
        ifOk(motor, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
        ifOk(
                motor,
                new DoubleSupplier[]{motor::getAppliedOutput, motor::getBusVoltage},
                (values) -> inputs.appliedVolts = values[0] * values[1]
        );
        ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAmps = value);
        inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
    }

    @Override
    public void setBrakeMode(boolean enable) {
        var newConfig = new SparkMaxConfig().idleMode(enable ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
        tryUntilOkAsync(5, () -> motor.configure(
                newConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
    }

    @Override
    public void setOpenLoop(double output) {
        motor.setVoltage(output);
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        var ffVolts = feedforward.calculate(velocityRadPerSec);
        controller.setReference(
                velocityRadPerSec,
                SparkBase.ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                ffVolts,
                SparkClosedLoopController.ArbFFUnits.kVoltage
        );
    }

    @Override
    public void setPosition(double positionRad) {
        double setpoint = MathUtil.inputModulus(positionRad, 0.0, 2 * Math.PI);
        controller.setReference(setpoint, SparkBase.ControlType.kPosition);
    }
}
