package frc.robot.subsystems.gamePieceVision;

import frc.robot.BuildConstants;

public class GamePieceVisionConstants {

    public static final double halfFOVRad = Math.PI / 3;
    public static final double maxRangeMeters = 5.0;


    static GamePieceVisionIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new GamePieceVisionIOLimelight("limelight");
            case SIM -> new GamePieceVisionIOSim();
            case REPLAY -> null;
        };
    }
}
