package frc.robot.factories.auto;

import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;

import java.util.Optional;
import java.util.function.Supplier;

public class AutoInitFactory {
    public static Command get(AutoRoutine routine, String autoName, Supplier<Optional<Pose2d>> initialPoseSupplier) {
        final var robotState = RobotState.get();

        return robotState.setPose(() -> {
            var initialPose = initialPoseSupplier.get();
            if (initialPose.isPresent()) {
                return initialPose.get();
            } else {
                var msg = "No initial pose for the first trajectory in auto " + autoName + "!";
                if (RobotBase.isSimulation())
                    throw new RuntimeException(msg);
                else
                    DriverStation.reportError(msg, false);

                routine.kill();
                return robotState.getPose();
            }
        }).withName("Auto Init");
    }
}
