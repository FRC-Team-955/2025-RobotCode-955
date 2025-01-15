package frc.robot.subsystems.drive.old;

import frc.robot.dashboard.DashboardSubsystem;
import frc.robot.dashboard.TuningDashboardAnglularVelocity;
import frc.robot.dashboard.TuningDashboardBoolean;
import frc.robot.dashboard.TuningDashboardPIDController;
import frc.robot.subsystems.drive.temp.DriveConstants;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public class DriveDashboard {
    static final TuningDashboardBoolean disableDriving = new TuningDashboardBoolean(
            DashboardSubsystem.DRIVE, "Disable Driving",
            false
    );
    static final TuningDashboardBoolean disableVision = new TuningDashboardBoolean(
            DashboardSubsystem.DRIVE, "Disable Vision",
            false
    );
    static final TuningDashboardBoolean disableGyro = new TuningDashboardBoolean(
            DashboardSubsystem.DRIVE, "Disable Gyro",
            false
    );

    static final TuningDashboardPIDController choreoFeedbackX = new TuningDashboardPIDController(
            DashboardSubsystem.DRIVE, "Choreo X PID",
            DriveConstants.driveConfig.choreoFeedbackXY()
    );
    static final TuningDashboardPIDController choreoFeedbackY = new TuningDashboardPIDController(
            DashboardSubsystem.DRIVE, "Choreo Y PID",
            DriveConstants.driveConfig.choreoFeedbackXY()
    );
    static final TuningDashboardPIDController choreoFeedbackTheta = new TuningDashboardPIDController(
            DashboardSubsystem.DRIVE, "Choreo Theta PID",
            DriveConstants.driveConfig.choreoFeedbackTheta(),
            (pid) -> pid.enableContinuousInput(-Math.PI, Math.PI)
    );
    static final TuningDashboardPIDController pointTowardsController = new TuningDashboardPIDController(
            DashboardSubsystem.DRIVE, "Point Towards PID",
            DriveConstants.driveConfig.pointTowardsController(),
            (pid) -> pid.enableContinuousInput(-Math.PI, Math.PI)
    );

    static final TuningDashboardAnglularVelocity characterizationSpeed = new TuningDashboardAnglularVelocity(DashboardSubsystem.DRIVE, "Wheel Radius Characterization Rotation Speed", RadiansPerSecond.of(1.0));
}
