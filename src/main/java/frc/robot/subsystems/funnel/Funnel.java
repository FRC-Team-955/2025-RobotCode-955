package frc.robot.subsystems.funnel;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.motor.MotorSubsystem;
import frc.lib.motor.RequestTolerances;
import frc.lib.motor.RequestType;
import frc.robot.RobotMechanism;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.funnel.FunnelConstants.createIO;
import static frc.robot.subsystems.funnel.FunnelTuning.*;

public class Funnel extends MotorSubsystem<Funnel.Goal> {
    private final RobotMechanism robotMechanism = RobotMechanism.get();


    @RequiredArgsConstructor
    @Getter
    public enum Goal implements GoalInterface {
        IDLE(() -> 0, RequestType.VoltageVolts),
        INTAKE_ALTERNATE(() -> Timer.getTimestamp() % 1.0 < 0.92 ? intakeGoalSetpoint.get() : -intakeGoalSetpoint.get(), RequestType.VelocityRadPerSec),
        EJECT_ALTERNATE(() -> Timer.getTimestamp() % 1.0 < 0.86 ? ejectGoalSetpoint.get() : -ejectGoalSetpoint.get(), RequestType.VelocityRadPerSec),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier value;
        private final RequestType type;
    }

    private static Funnel instance;

    public static Funnel get() {
        if (instance == null)
            synchronized (Funnel.class) {
                instance = new Funnel();
            }

        return instance;
    }

    private Funnel() {
        super(
                "Funnel",
                RequestTolerances.defaults(),
                createIO(),
                Goal.IDLE
        );
    }

    @Override
    public void periodicBeforeCommands() {
        super.periodicBeforeCommands();

        robotMechanism.funnel.beltLigament.setAngle(Units.radiansToDegrees(-inputs.positionRad));

        velocityGainsTunable.ifChanged(io::setVelocityPIDF);
    }
}
