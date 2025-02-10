package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.*;
import static frc.robot.util.SparkUtil.tryUntilOkAsync;

public class ElevatorIOSparkMax extends ElevatorIO {
    // Hardware objects
    private final SparkMax leadMotor;
    private final SparkMax followMotor;
    private final RelativeEncoder leadEncoder;
    private final RelativeEncoder followEncoder;
    private final SparkMaxConfig leadConfig;
    private final SparkMaxConfig followConfig;

    private final DigitalInput limitSwitch;

    // Closed loop controllers
    private final ProfiledPIDController controller;
    private final ElevatorFeedforward ff;

    // Connection debouncers
    private final Debouncer leadConnectedDebounce = new Debouncer(0.5);
    private final Debouncer followConnectedDebounce = new Debouncer(0.5);

    private boolean closedLoop = true;
    private double setpointVelocityRadPerSec;

    public ElevatorIOSparkMax(
            int leadCanID,
            int followCanID,
            int limitSwitchID,
            boolean leaderInverted
    ) {
        leadMotor = new SparkMax(leadCanID, SparkLowLevel.MotorType.kBrushless);
        followMotor = new SparkMax(followCanID, SparkLowLevel.MotorType.kBrushless);
        leadEncoder = leadMotor.getEncoder();
        followEncoder = followMotor.getEncoder();

        limitSwitch = new DigitalInput(limitSwitchID);

        controller = gains.toProfiledPID(new TrapezoidProfile.Constraints(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSquared));
        ff = gains.toElevatorFF();

        // Configure motors
        leadConfig = new SparkMaxConfig();
        followConfig = new SparkMaxConfig();

        leadConfig
                .inverted(leaderInverted)
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0);
        leadConfig
                .encoder
                .positionConversionFactor(2 * Math.PI) // Rotor Rotations -> Rotor Radians
                .velocityConversionFactor((2 * Math.PI ) / 60.0) // Rotor RPM -> Rotor Rad/Sec
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

        followConfig.apply(leadConfig).follow(leadMotor, true);

        tryUntilOk(5, () -> leadMotor.configure(
                leadConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
        tryUntilOk(5, () -> followMotor.configure(
                followConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
        tryUntilOk(5, () -> leadEncoder.setPosition(0.0));
        tryUntilOk(5, () -> followEncoder.setPosition(0.0));
    }

    @Override
    public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
        if (closedLoop) {
            leadMotor.setVoltage(
                    controller.calculate(leadEncoder.getPosition())
                            + ff.calculate(setpointVelocityRadPerSec));
        } else {
            controller.reset(new TrapezoidProfile.State(leadEncoder.getPosition(), leadEncoder.getVelocity()));
        }
        // Update drive inputs
        sparkStickyFault = false;
        ifOk(leadMotor, leadEncoder::getPosition, (value) -> inputs.leaderPositionRad = value);
        ifOk(leadMotor, leadEncoder::getVelocity, (value) -> inputs.leaderVelocityRadPerSec = value);
        ifOk(
                leadMotor,
                new DoubleSupplier[]{leadMotor::getAppliedOutput, leadMotor::getBusVoltage},
                (values) -> inputs.leaderAppliedVolts = values[0] * values[1]
        );
        ifOk(leadMotor, leadMotor::getOutputCurrent, (value) -> inputs.leaderCurrentAmps = value);
        inputs.leaderConnected = leadConnectedDebounce.calculate(!sparkStickyFault);

        sparkStickyFault = false;
        ifOk(followMotor, followEncoder::getPosition, (value) -> inputs.followerPositionRad = value);
        ifOk(followMotor, followEncoder::getVelocity, (value) -> inputs.followerVelocityRadPerSec = value);
        ifOk(
                followMotor,
                new DoubleSupplier[]{followMotor::getAppliedOutput, followMotor::getBusVoltage},
                (values) -> inputs.followerAppliedVolts = values[0] * values[1]
        );
        ifOk(followMotor, followMotor::getOutputCurrent, (value) -> inputs.followerCurrentAmps = value);
        inputs.followerConnected = followConnectedDebounce.calculate(!sparkStickyFault);

        inputs.limitSwitchConnected = true;
        inputs.limitSwitchTriggered = limitSwitch.get();
    }

    @Override
    public void setBrakeMode(boolean enable) {
        var newConfig = new SparkMaxConfig().idleMode(enable ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
        tryUntilOkAsync(5, () -> leadMotor.configure(
                newConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
        tryUntilOkAsync(5, () -> followMotor.configure(
                newConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
    }

    @Override
    public void setOpenLoop(double output) {
        closedLoop = false;
        leadMotor.setVoltage(output);
    }

    @Override
    public void setClosedLoop(double positionRad, double velocityRadPerSec) {
        closedLoop = true;
        controller.setGoal(new TrapezoidProfile.State(positionRad, velocityRadPerSec));
        setpointVelocityRadPerSec = velocityRadPerSec;
    }

    @Override
    public void setEncoder(double positionRad) {
        tryUntilOk(5, () -> leadEncoder.setPosition(positionRad));
        tryUntilOk(5, () -> followEncoder.setPosition(positionRad));
    }
}
