package frc.lib.swerve.torque_based;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public record SwerveSetpoint(
        ChassisSpeeds robotRelativeSpeeds,
        SwerveModuleState[] moduleStates,
        DriveFeedforwards feedforwards
) {}