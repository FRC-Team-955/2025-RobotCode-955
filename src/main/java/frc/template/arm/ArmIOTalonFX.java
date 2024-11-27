package frc.template.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.util.Units;

public class ArmIOTalonFX extends ArmIO {
    private final TalonFX motor;

    private final StatusSignal<Double> position;
    private final StatusSignal<Double> velocity;
    private final StatusSignal<Double> appliedVolts;
    private final StatusSignal<Double> current;

    private double gearRatio = 1.0;

    public ArmIOTalonFX(int canID, boolean inverted) {
        motor = new TalonFX(canID);
        var config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        motor.getConfigurator().apply(config);

        position = motor.getPosition();
        velocity = motor.getVelocity();
        appliedVolts = motor.getMotorVoltage();
        current = motor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);
        inputs.positionRad = Units.rotationsToRadians(position.getValue()) / gearRatio;
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocity.getValue()) / gearRatio;
        inputs.appliedVolts = appliedVolts.getValue();
        inputs.currentAmps = current.getValue();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setSetpoint(double setpointPositionRad, double ffVolts) {
        motor.setControl(new PositionVoltage(
                Units.radiansToRotations(setpointPositionRad * gearRatio),
                0.0,
                true,
                ffVolts,
                0,
                false,
                false,
                false
        ));
    }

    @Override
    public void setPosition(double currentPositionRad) {
        motor.setPosition(Units.radiansToRotations(currentPositionRad * gearRatio));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        var config = new MotorOutputConfigs();
        config.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        motor.getConfigurator().apply(config);
    }

    @Override
    public void configurePID(PIDConstants pidConstants) {
        var config = new Slot0Configs();
        config.kP = pidConstants.kP;
        config.kI = pidConstants.kI;
        config.kD = pidConstants.kD;
        motor.getConfigurator().apply(config);
    }

    @Override
    public void setGearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
    }
}
