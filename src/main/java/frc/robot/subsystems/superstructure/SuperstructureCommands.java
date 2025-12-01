package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controller;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gamePieceVision.GamePieceVision;

public abstract class SuperstructureCommands {
    protected final RobotState robotState = RobotState.get();
    protected final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    protected final Controller controller = Controller.get();

    protected final Superstructure superstructure = Superstructure.get();
    protected final AprilTagVision aprilTagVision = AprilTagVision.get();
    protected final Drive drive = Drive.get();
    protected final GamePieceVision gamePieceVision = GamePieceVision.get();

    public abstract Command create();

//
//    protected Command waitUntilIntakeTriggered() {
//        return Commands.waitUntil(superstructure::isIntakeTriggered);
//    }


    protected Command rumble() {
        return controller.rumble(0.5, 0.5);
    }

    protected Command shake() {
        return drive.runRobotRelative(() -> Timer.getTimestamp() % 0.25 < 0.125
                ? new ChassisSpeeds(-0.05, -0.05, -0.3)
                : new ChassisSpeeds(0.05, 0.05, 0.3));
    }
}
