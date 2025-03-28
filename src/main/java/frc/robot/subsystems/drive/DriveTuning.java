package frc.robot.subsystems.drive;

import frc.robot.util.PIDF;
import frc.robot.util.network.LoggedTunableNumber;

import static frc.robot.subsystems.drive.DriveConstants.*;

public class DriveTuning {
    public static final LoggedTunableNumber characterizationSpeedRadPerSec = new LoggedTunableNumber("Drive/WheelRadiusCharacterizationSpeedRadPerSec", 1.0);

    public static final PIDF.Tunable moduleDriveGainsTunable = moduleConfig.driveGains().tunable("Drive/ModuleDrive");
    public static final PIDF.Tunable moduleTurnGainsTunable = moduleConfig.turnGains().tunable("Drive/ModuleTurn");

    public static final PIDF.Tunable moveToLinearTunable = moveToLinear.tunable("Drive/MoveToLinear");
    public static final LoggedTunableNumber moveToLinearMaxVelocityTunable = new LoggedTunableNumber("Drive/MoveToLinear/MaxVelocityMetersPerSecond", moveToLinearConstraintsMeters.maxVelocity);
    public static final LoggedTunableNumber moveToLinearMaxAccelerationTunable = new LoggedTunableNumber("Drive/MoveToLinear/MaxAccelerationMetersPerSecondSquared", moveToLinearConstraintsMeters.maxAcceleration);

    public static final PIDF.Tunable moveToAngularTunable = moveToAngular.tunable("Drive/MoveToAngular");
    public static final LoggedTunableNumber moveToAngularMaxVelocityTunable = new LoggedTunableNumber("Drive/MoveToAngular/MaxVelocityRadPerSecond", moveToAngularConstraintsRad.maxVelocity);
    public static final LoggedTunableNumber moveToAngularMaxAccelerationTunable = new LoggedTunableNumber("Drive/MoveToAngular/MaxAccelerationRadPerSecondSquared", moveToAngularConstraintsRad.maxAcceleration);
}
