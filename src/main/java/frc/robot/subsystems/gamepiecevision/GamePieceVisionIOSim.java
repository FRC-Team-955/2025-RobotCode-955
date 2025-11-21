package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotState;
import frc.robot.subsystems.gamepiecevision.GamePieceVisionIO;
import lombok.Getter;
import org.ironmaple.simulation.SimulatedArena;

import java.util.List;


public class GamePieceVisionIOSim extends GamePieceVisionIO {
    public static SimulatedArena arena = SimulatedArena.getInstance();
    private final RobotState robotState = RobotState.get();

    public GamePieceVisionIOSim() {
    }

    @Override
    public void updateInputs(GamePieceVisionIOInputs inputs) {

        inputs.connected = true;
//
        if (arena == null) {
            inputs.visible = false;
            return;
        }
        Pose2d robotPos = robotState.getPose();
        Translation2d robotXY = robotPos.getTranslation();
        double robotYawRad = robotState.getRotation().getRadians() + Math.PI;
        List<Pose3d> corals = arena.getGamePiecesPosesByType("Coral");
        if (corals.isEmpty()) {
            inputs.visible = false;
            return;
        }

        Pose3d nearestCoral = null;
        double bestDist = Double.MAX_VALUE;

        for (Pose3d coral : corals) {
            Translation2d coralXY = new Translation2d(coral.getX(),
                    coral.getY());
            double dist = coralXY.getDistance(robotXY);
            if (dist < bestDist) {
                bestDist = dist;
                nearestCoral = coral;
            }
        }


        double maxRangeMeters = 5.0;
        if (nearestCoral == null || bestDist > maxRangeMeters) {
            inputs.visible = false;
            return;
        }


        Pose2d coral2 = nearestCoral.toPose2d();

        double dx = coral2.getX() - robotXY.getX();
        double dy = coral2.getY() - robotXY.getY();

        double angleToCoral = Math.atan2(dy, dx);
        double yawRad = angleToCoral - robotYawRad;
        yawRad = Math.atan2(Math.sin(yawRad), Math.cos(yawRad));

        inputs.coralPos = coral2;

        inputs.coralYaw = new Rotation2d(yawRad);
        inputs.coralPitch = new Rotation2d();
        if (Math.abs(yawRad) > GamePieceVisionConstants.halfFOVRad) {
            inputs.visible = false;
            return;
        }
        inputs.visible = true;


    }
}