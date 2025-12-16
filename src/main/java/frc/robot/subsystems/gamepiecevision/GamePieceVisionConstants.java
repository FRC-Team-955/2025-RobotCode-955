package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import frc.robot.BuildConstants;

public class GamePieceVisionConstants {
    public static double halfFOVRad = Math.PI / 3;

    static GamePieceVisionIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new GamePieceVisionIOLimelight("*");
            case SIM -> new GamePieceVisionIOSim();
            case REPLAY -> new GamePieceVisionIO();
        };
    }
}
