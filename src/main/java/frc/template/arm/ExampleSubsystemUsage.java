package frc.template.arm;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.template.absoluteencoder.AbsoluteEncoderIO;
import frc.template.absoluteencoder.AbsoluteEncoderIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Radians;

public final class ExampleSubsystemUsage extends SubsystemBase {
    private static final double ARM_SETPOINT_TOLERANCE = Units.degreesToRadians(7);

    private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
    private final ArmIO armIO;

    private final AbsoluteEncoderIOInputsAutoLogged absoluteEncoderInputs = new AbsoluteEncoderIOInputsAutoLogged();
    private final AbsoluteEncoderIO absoluteEncoderIO;

    private final ArmFeedforward armFeedforward;
    private Angle armInitialPosition;
    private Double armSetpointRad;

    private final double ARM_VISUALIZATION_SIZE = 6;
    private final double ARM_VISUALIZATION_LENGTH = 2.5;
    private final Color ARM_VISUALIZATION_COLOR = Color.kOrange;
    public final Mechanism2d mechanism = new Mechanism2d(ARM_VISUALIZATION_SIZE, ARM_VISUALIZATION_SIZE, new Color8Bit(Color.kGray));
    public final MechanismLigament2d ligament = mechanism.getRoot("Root", ARM_VISUALIZATION_SIZE / 2, ARM_VISUALIZATION_SIZE / 2).append(new MechanismLigament2d("Arm", ARM_VISUALIZATION_LENGTH, 0.0, 4.0, new Color8Bit(ARM_VISUALIZATION_COLOR)));

    /**
     * @param gearRatio >1 means a reduction, <1 means a upduction
     */
    private ExampleSubsystemUsage(
            ArmIO armIO,
            ArmFeedforward armFeedforward,
            PIDConstants armFeedback,
            double gearRatio,
            AbsoluteEncoderIO absoluteEncoderIO,
            Angle absoluteEncoderOffset
    ) {
        this.absoluteEncoderIO = absoluteEncoderIO;
        this.armIO = armIO;
        this.armFeedforward = armFeedforward;
        this.armInitialPosition = null;

        armIO.configurePID(armFeedback);
        armIO.setGearRatio(gearRatio);

        absoluteEncoderIO.setGearRatio(gearRatio);
        absoluteEncoderIO.setOffset(absoluteEncoderOffset.in(Radians));
    }

    /**
     * @param gearRatio       >1 means a reduction, <1 means a upduction
     * @param initialPosition Initial position of the arm. 0 means parallel to the ground.
     */
    private ExampleSubsystemUsage(
            ArmIO armIO,
            ArmFeedforward armFeedforward,
            PIDConstants armFeedback,
            double gearRatio,
            Angle initialPosition
    ) {
        this.armIO = armIO;
        this.armFeedforward = armFeedforward;
        this.absoluteEncoderIO = null;
        this.armInitialPosition = initialPosition;

        armIO.configurePID(armFeedback);
        armIO.setGearRatio(gearRatio);
        armIO.setPosition(initialPosition.in(Radians));

        // Immediately start closed loop with the starting position as our setpoint
        armSetpointRad = initialPosition.in(Radians);
    }

    public void periodic() {
        if (absoluteEncoderIO != null) {
            absoluteEncoderIO.updateInputs(absoluteEncoderInputs);
            Logger.processInputs("Inputs/ExampleSubsystem", absoluteEncoderInputs);

            // initialize initialPosition if we have an absolute encoder
            // must go before arm IO update so that the IO position is set
            if (armInitialPosition == null) {
                armInitialPosition = Radians.of(absoluteEncoderInputs.absolutePositionRad);
                armIO.setPosition(armInitialPosition.in(Radians));

                // Immediately start closed loop with the initial position as our setpoint
                armSetpointRad = armInitialPosition.in(Radians);
            }
        }

        armIO.updateInputs(armInputs);
        Logger.processInputs("Inputs/ExampleSubsystem", armInputs);

        Logger.recordOutput("ExampleSubsystem/ClosedLoop", armSetpointRad != null);
        if (armSetpointRad != null) {
            Logger.recordOutput("ExampleSubsystem/Setpoint", armSetpointRad);

            if (DriverStation.isEnabled()) {
                var ffVolts = armFeedforward.calculate(armSetpointRad, 0);
                Logger.recordOutput("ExampleSubsystem/FFVolts", ffVolts);
                armIO.setSetpoint(armSetpointRad, ffVolts);
            }
        }

        ligament.setAngle(Units.radiansToDegrees(armInputs.positionRad));
    }

    public Angle getPosition() {
        return Radians.of(armInputs.positionRad);
    }

    public void setPercent(double percent) {
        armIO.setVoltage(percent * 12);
        armSetpointRad = null;
    }

    /**
     * 0 means parallel to the ground
     */
    public void setSetpoint(Angle setpoint) {
        armSetpointRad = setpoint.in(Radians);
    }

    /**
     * Note: the tolerance is currently 5 degrees. If you need more
     * precision edit this class to take the tolerance as a parameter.
     */
    public boolean atSetpoint() {
        return Math.abs(armInputs.positionRad - armSetpointRad) <= ARM_SETPOINT_TOLERANCE;
    }

    public void stop() {
        armIO.setVoltage(0);
        armSetpointRad = null;
    }

    /**
     * Tells the encoder the current position is the initial position.
     */
    public void setPosition() {
        setPosition(armInitialPosition);
    }

    /**
     * Tells the encoder the current position is the one specified.
     */
    public void setPosition(Angle position) {
        armIO.setPosition(position.in(Radians));
    }

    public void setBreakMode(boolean enabled) {
        armIO.setBrakeMode(enabled);
    }
}
