package frc.robot.subsystems.drive.goals;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.Supplier;

public class VelocityRobotRelativeGoal {
    private static Supplier<ChassisSpeeds> chassisSpeedsSupplier = ChassisSpeeds::new;

    public static void initialize(Supplier<ChassisSpeeds> newChassisSpeedsSupplier) {
        chassisSpeedsSupplier = newChassisSpeedsSupplier;
    }

    public static ChassisSpeeds get() {
        return chassisSpeedsSupplier.get();
    }
}