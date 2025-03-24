package frc.robot.subsystems.gamepiecevision;

import frc.robot.Constants;

public class GamePieceVisionConstants {
    public static GamePieceVisionIO createIO() {
        if (Constants.isReplay) {
            return new GamePieceVisionIO();
        }

        return switch (Constants.identity) {
            case COMPBOT -> new GamePieceVisionIOLimelight("limelight");
            case ALPHABOT -> new GamePieceVisionIO();
            case SIMBOT -> new GamePieceVisionIOSim();
        };
    }
}
