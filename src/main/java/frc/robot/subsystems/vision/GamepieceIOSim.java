package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotState;

public class GamepieceIOSim extends GamepieceIO {
    RobotState robotState = RobotState.get();

    public GamepieceIOSim() {}

    @Override
    public void updateInputs(GamepieceIO.GamepieceIOInputs inputs) {
        inputs.connected = true;
        inputs.latestTargetObservation = new TargetObservation(
                new Rotation2d(0),
                new Rotation2d(0),
                new Translation2d(27, 13).minus(robotState.getTranslation()),
                true
        );
    }
}
