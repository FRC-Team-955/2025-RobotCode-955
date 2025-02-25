package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
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
    private static final double epsilon = 1E-6;

    public static boolean shouldFlip() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    public static Rotation2d flipIfNeeded(Rotation2d rotation2d) {
        return shouldFlip()
                ? rotation2d.plus(Rotation2d.kPi)
                : rotation2d;
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
        return sysIdRoutine(name, voltageConsumer, start, subsystem, null, null, null);
    }

    /**
     * start and end should be used to set the goal to characterization
     */
    public static SysIdRoutine sysIdRoutine(
            String name,
            Consumer<Voltage> voltageConsumer,
            Runnable start,
//            Runnable end,
            Subsystem subsystem,
            Velocity<VoltageUnit> rampRate,
            Voltage stepVoltage,
            Time timeout
    ) {
        // Java forces us to do this if we want to use the variable in the lambda
        var ref = new Object() {
            boolean hasStarted = false;
        };
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        rampRate,
                        stepVoltage,
                        timeout,
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

    public static boolean epsilonEquals(double a, double b) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(ChassisSpeeds s1, ChassisSpeeds s2) {
        return epsilonEquals(s1.vxMetersPerSecond, s2.vxMetersPerSecond)
                && epsilonEquals(s1.vyMetersPerSecond, s2.vyMetersPerSecond)
                && epsilonEquals(s1.omegaRadiansPerSecond, s2.omegaRadiansPerSecond);
    }

    public static boolean greaterThanEpsilon(double a) {
        return a > epsilon;
    }

    public static int findArrayIndexWithClosestValue(double targetValue, double[] array) {
        double smallestDiff = Double.MAX_VALUE;
        int smallestIndex = 0;
        for (int i = 0; i < array.length; i++) {
            double diff = Math.abs(array[i] - targetValue);
            if (diff < smallestDiff) {
                smallestDiff = diff;
                smallestIndex = i;
            }
        }
        return smallestIndex;
    }
}