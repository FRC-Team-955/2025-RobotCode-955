// https://github.com/mjansen4857/pathplanner/blob/main/pathplannerlib/src/main/java/com/pathplanner/lib/util/swerve/SwerveSetpoint.java

package frc.robot.util.swerve;

import frc.robot.util.DriveFeedforwards;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * A setpoint for a swerve drivetrain, containing robot-relative chassis speeds and individual
 * module states
 *
 * @param robotRelativeSpeeds Robot-relative chassis speeds
 * @param moduleStates Array of individual swerve module states. These will be in FL, FR, BL, BR
 *     order.
 * @param feedforwards Feedforwards for each module's drive motor. The arrays in this record will be
 *     in FL, FR, BL, BR order.
 */
public record SwerveSetpoint(
        ChassisSpeeds robotRelativeSpeeds,
        SwerveModuleState[] moduleStates,
        DriveFeedforwards feedforwards) {}