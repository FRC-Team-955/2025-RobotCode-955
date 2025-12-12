package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

public class SuperstructureIOSim extends SuperstructureIO {
    private Superstructure superstructure = null;

    private boolean added = false;

    @Override
    public void updateInputs(SuperstructureIOInputs inputs) {
        if (superstructure == null) {
            superstructure = Superstructure.get();
        }
        if (superstructure.getGoal() == Superstructure.Goal.AUTO_INTAKE_SEARCHING || superstructure.getGoal() == Superstructure.Goal.AUTO_INTAKE_SEARCHING_FOR_STALE || superstructure.getGoal() == Superstructure.Goal.AUTO_INTAKE_INTAKING) {
            if (!added) {
                SimulatedArena.getInstance().clearGamePieces();
                SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(new Pose2d(0.7 + 3.0 * Math.random(), 0.7 + 2.5 * Math.random(), new Rotation2d())));
                added = true;
            }
        } else {
            if (added) {
                SimulatedArena.getInstance().clearGamePieces();
                added = false;
            }
        }
    }
}
