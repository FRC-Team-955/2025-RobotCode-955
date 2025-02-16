package frc.robot.subsystems.drive;

import frc.robot.util.PIDF;
import frc.robot.util.network.LoggedTunableNumber;

import static frc.robot.subsystems.drive.DriveConstants.moduleConfig;

public class DriveTuning {
    public static final LoggedTunableNumber characterizationSpeedRadPerSec = new LoggedTunableNumber("Drive/Wheel Radius Characterization Rotation Speed (rad per sec)", 1.0);

    public static final PIDF.Tunable moduleDriveGainsTunable = moduleConfig.driveGains().tunable("Drive/ModuleDrive");
    public static final PIDF.Tunable moduleTurnGainsTunable = moduleConfig.turnGains().tunable("Drive/ModuleTurn");
}
