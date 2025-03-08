//package frc.robot.subsystems.indexer;
//
//import edu.wpi.first.math.system.plant.DCMotor;
//import frc.robot.Constants;
//import frc.robot.subsystems.rollers.RollersConfig;
//import frc.robot.subsystems.rollers.RollersIO;
//import frc.robot.subsystems.rollers.RollersIOSim;
//import frc.robot.util.PIDF;
//
//public class IndexerConstants {
//    protected static RollersIO createRollersIO() {}
//    protected static final RollersIO rollersIO = Constants.isReplay
//            ? new RollersIO()
//            : switch (Constants.identity) {
//        case COMPBOT -> new RollersIO();
//        case SIMBOT -> new RollersIOSim(
//                new RollersConfig(
//                        false,
//                        true,
//                        40,
//                        3,
//                        PIDF.ofP(1),
//                        PIDF.ofPSVA(1, 0, 1, 1)
//                ),
//                0.01,
//                DCMotor.getNEO(1)
//        );
//        case ALPHABOT -> new RollersIO();
//    };
//
//    public static final double setpointToleranceRadPerSec = 0;
//}