package frc.robot.subsystems.drive.goals;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lombok.Setter;

import java.util.function.Supplier;

public class VelocityRobotRelativeGoal {
    @Setter
    private static Supplier<ChassisSpeeds> chassisSpeedsSupplier = ChassisSpeeds::new;

    public static ChassisSpeeds get() {
        return chassisSpeedsSupplier.get();
    }
}