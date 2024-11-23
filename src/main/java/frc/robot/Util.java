package frc.robot;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;

public class Util {
    public static boolean shouldFlip() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    public static Pose2d flipIfNeeded(Pose2d pose) {
        if (shouldFlip())
            return GeometryUtil.flipFieldPose(pose);
        else
            return pose;
    }

    public static Translation2d flipIfNeeded(Translation2d pos) {
        if (shouldFlip())
            return GeometryUtil.flipFieldPosition(pos);
        else
            return pos;
    }

    public static double angle(Translation2d from, Translation2d to) {
        return Math.atan2(from.getY() - to.getY(), from.getX() - to.getX());
    }

    /**
     * start and end should be used to set the goal to characterization
     */
    public static SysIdRoutine sysIdRoutine(
            String name,
            Consumer<Measure<Voltage>> voltageConsumer,
            Runnable start,
            Runnable end,
            Subsystem subsystem
    ) {
        // Java forces us to do this if we want to use the variable in the lambda
        var ref = new Object() {
            boolean hasStarted = false;
        };
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> {
                            Logger.recordOutput(name + "/SysIdState", state.toString());
                            if (!ref.hasStarted && state != SysIdRoutineLog.State.kNone) {
                                start.run();
                                ref.hasStarted = true;
                            } else if (ref.hasStarted && state == SysIdRoutineLog.State.kNone) {
                                end.run();
                                ref.hasStarted = false;
                            }
                        }
                ),
                new SysIdRoutine.Mechanism(
                        voltageConsumer,
                        null,
                        subsystem
                )
        );
    }
}
