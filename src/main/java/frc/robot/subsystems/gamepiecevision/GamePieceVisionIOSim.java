package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.ModuleIOSim;
import org.ironmaple.simulation.SimulatedArena;

import java.util.LinkedList;
import java.util.List;

import static frc.robot.subsystems.gamepiecevision.GamePieceVisionConstants.camera;
import static frc.robot.subsystems.gamepiecevision.GamePieceVisionConstants.coralHeightMeters;

public class GamePieceVisionIOSim extends GamePieceVisionIO {
    public GamePieceVisionIOSim() {
    }

    @Override
    public void updateInputs(GamePieceVisionIOInputs inputs) {
        inputs.connected = true;

        List<TargetObservation> targetObservations = new LinkedList<>();

        Pose3d robotPose = new Pose3d(ModuleIOSim.driveSimulation.getSimulatedDriveTrainPose());
        Pose3d[] coralPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
        for (var coralPose : coralPoses) {
            // Ignore coral not on ground
            if (coralPose.getZ() > coralHeightMeters / 2.0 + 0.01) {
                continue;
            }

            Transform3d robotToCoral = new Transform3d(robotPose, coralPose);
            Transform3d camToCoral = camera.robotToCamera().inverse().plus(robotToCoral);

            double yaw = Math.atan2(camToCoral.getY(), camToCoral.getX());
            // rotation order is yaw-pitch (Tait-Bryan angles without roll)
            // that's why we use hypotenuse of x-y triangle (xyDist) when calculating pitch instead of x
            double xyDist = Math.sqrt(camToCoral.getX() * camToCoral.getX() + camToCoral.getY() * camToCoral.getY());
            double pitch = -Math.atan2(camToCoral.getZ(), xyDist);

            if (Math.abs(yaw) > camera.horizontalFovRad() / 2.0 || Math.abs(pitch) > camera.verticalFovRad() / 2.0) {
                continue;
            }

            targetObservations.add(new TargetObservation(Timer.getFPGATimestamp(), yaw, pitch));
        }

        inputs.targetObservations = targetObservations.toArray(TargetObservation[]::new);
    }
}
