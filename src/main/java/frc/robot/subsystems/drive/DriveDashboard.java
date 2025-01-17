package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class DriveDashboard {
    static final LoggedNetworkBoolean disableDriving = new LoggedNetworkBoolean("Drive/Disable Driving", false);
    static final LoggedNetworkBoolean disableVision = new LoggedNetworkBoolean("Drive/Disable Vision", false);
    static final LoggedNetworkBoolean disableGyro = new LoggedNetworkBoolean("Drive/Disable Gyro", false);
}
