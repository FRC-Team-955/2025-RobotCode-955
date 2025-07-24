package frc.robot.subsystems.gamepiecevision;

import frc.robot.BuildConstants;

public class GamePieceVisionConstants {
    public static GamePieceVisionIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new GamePieceVisionIOLimelight("limelight");
            case SIM -> new GamePieceVisionIOSim();
            case REPLAY -> new GamePieceVisionIO();
        };
    }
}
