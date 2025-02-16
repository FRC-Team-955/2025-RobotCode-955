package frc.robot.subsystems.elevator;

import frc.robot.util.PIDF;
import frc.robot.util.network.LoggedTunableNumber;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

public class ElevatorTuning {
    public static final PIDF.Tunable gainsTunable = gains.tunable("Elevator/Gains");

    public static final LoggedTunableNumber maxVelocityMetersPerSecondTunable =
            new LoggedTunableNumber("Elevator/MaxVelocityMetersPerSecond", maxVelocityMetersPerSecond);
    public static final LoggedTunableNumber maxAccelerationMetersPerSecondSquaredTunable =
            new LoggedTunableNumber("Elevator/MaxAccelerationMetersPerSecondSquared", maxAccelerationMetersPerSecondSquared);
}