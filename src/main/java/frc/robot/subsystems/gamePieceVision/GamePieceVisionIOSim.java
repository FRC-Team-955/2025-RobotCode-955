package frc.robot.subsystems.gamePieceVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.ModuleIOSim;
import org.ironmaple.simulation.SimulatedArena;

import java.util.List;

import static frc.robot.subsystems.gamePieceVision.GamePieceVisionConstants.maxRangeMeters;

public class GamePieceVisionIOSim extends GamePieceVisionIO {
    private final SimulatedArena arena = SimulatedArena.getInstance();
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
        Pose2d robotPos = ModuleIOSim.driveSimulation.getSimulatedDriveTrainPose();
        Translation2d robotXY = robotPos.getTranslation();
        double robotYawRad = robotState.getRotation().getRadians();
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


        if (nearestCoral == null || bestDist > maxRangeMeters) {
            inputs.visible = false;
            return;
        }


        Pose2d coral2 = nearestCoral.toPose2d();

        double dx = coral2.getX() - robotXY.getX();
        double dy = coral2.getY() - robotXY.getY();
        Translation2d coralFieldRel = coral2.getTranslation().minus(robotXY);
        Translation2d coralRobotRel =
                coralFieldRel.rotateBy(robotPos.getRotation().unaryMinus());
        inputs.coralRobotRelative = coralRobotRel;

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





