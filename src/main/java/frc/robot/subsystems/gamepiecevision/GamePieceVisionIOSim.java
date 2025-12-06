package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.ModuleIOSim;
import org.ironmaple.simulation.SimulatedArena;

import java.util.LinkedList;
import java.util.List;

import static frc.robot.subsystems.gamepiecevision.GamePieceVisionConstants.camera;

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
            Transform3d robotToCoral = new Transform3d(robotPose, coralPose);
            Transform3d camToCoral = camera.robotToCamera().inverse().plus(robotToCoral);

            double tx = -Math.atan2(camToCoral.getY(), camToCoral.getX());
            double ty = Math.atan2(camToCoral.getZ(), camToCoral.getX());

            if (Math.abs(tx) > camera.horizontalFovRad() || Math.abs(ty) > camera.verticalFovRad()) {
                continue;
            }

            targetObservations.add(new TargetObservation(Timer.getFPGATimestamp(), Rotation2d.fromRadians(tx), Rotation2d.fromRadians(ty)));
        }

        inputs.targetObservations = targetObservations.toArray(TargetObservation[]::new);
    }
}
