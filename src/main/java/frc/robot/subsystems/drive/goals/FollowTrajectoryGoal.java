package frc.robot.subsystems.drive.goals;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.Util;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.drive.DriveConstants.driveConfig;


public class FollowTrajectoryGoal {
    private static final RobotState robotState = RobotState.get();

    private static Trajectory<SwerveSample> trajectory = null;

    private static final Timer timer = new Timer();
    private static final PIDController choreoFeedbackX = driveConfig.choreoFeedbackXY().toPID();
    private static final PIDController choreoFeedbackY = driveConfig.choreoFeedbackXY().toPID();
    private static final PIDController choreoFeedbackOmega = driveConfig.choreoFeedbackOmega().toPIDWrapRadians();

    public static void initialize(Trajectory<SwerveSample> newTrajectory) {
        trajectory = newTrajectory;

        timer.stop();
        choreoFeedbackX.reset();
        choreoFeedbackY.reset();
        choreoFeedbackOmega.reset();
    }

    public static ChassisSpeeds get() {
        if (trajectory == null) {
            Util.error("Trajectory is null");
            return new ChassisSpeeds();
        }

        if (!timer.isRunning()) {
            timer.restart();
        }

        Logger.recordOutput("Drive/Trajectory", trajectory.getPoses());

        var sampleOpt = trajectory.sampleAt(timer.get(), Util.shouldFlip());

        if (sampleOpt.isPresent()) {
            SwerveSample sample = sampleOpt.get();

            var currentPose = robotState.getPose();

            Logger.recordOutput("Drive/TrajectorySetpoint", sample.getPose());
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    sample.vx + choreoFeedbackX.calculate(currentPose.getX(), sample.x),
                    sample.vy + choreoFeedbackY.calculate(currentPose.getY(), sample.y),
                    sample.omega + choreoFeedbackOmega.calculate(currentPose.getRotation().getRadians(), sample.heading),
                    currentPose.getRotation() // Trajectories are absolute, don't flip
            );
        } else {
            return new ChassisSpeeds();
        }
    }
}
