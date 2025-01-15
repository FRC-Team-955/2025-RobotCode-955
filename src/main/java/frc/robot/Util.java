package frc.robot;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

import java.net.NetworkInterface;
import java.util.Arrays;
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
            Consumer<Voltage> voltageConsumer,
            Runnable start,
//            Runnable end,
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
//                                end.run();
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

    // https://github.com/FRCTeam2910/2024CompetitionRobot-Public/blob/main/src/main/java/frc/robot/util/MacAddressUtil.java
    public static String getMacAddress() {
        try {
            var networkInterface = NetworkInterface.getNetworkInterfaces();
            var macAddress = new StringBuilder();
            while (networkInterface.hasMoreElements()) {
                var tempInterface = networkInterface.nextElement(); // Instantiates the next element in our network interface
                if (tempInterface != null) {
                    byte[] mac = tempInterface.getHardwareAddress(); // Reads the MAC address from our NetworkInterface
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            // Formats our mac address by splitting it into two-character segments and hyphenating them
                            // (unless it is the final segment)
                            macAddress.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        System.out.println("Using interface " + tempInterface.getDisplayName() + " for mac address");
                        return macAddress.toString();
                    } else {
                        System.out.println("Address not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address not found");
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        return "";
    }

    public static double average(double... inputs) {
        return Arrays.stream(inputs).sum() / inputs.length;
    }

    public static void error(String msg) {
        if (RobotBase.isSimulation()) {
            throw new RuntimeException(msg);
        } else {
            DriverStation.reportError(msg, false);
        }
    }
}