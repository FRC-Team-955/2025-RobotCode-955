package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final RobotIdentity identity = RobotIdentity.determine();

    public static final Mode mode = Simulation.shouldReplay
            ? Mode.REPLAY
            : switch (identity) {
        case SIMBOT -> Mode.SIM;
        default -> Mode.REAL;
    };

    public static final boolean isReplay = mode ==Mode.REPLAY;

    public enum Mode {
        /**
         * Real robot
         */
        REAL,
        /**
         * Simulation
         */
        SIM,
        /**
         * Log replay
         */
        REPLAY
    }

    public static final class Simulation {
        public static final boolean shouldReplay = false;
        public static final RobotIdentity replayIdentity = RobotIdentity.COMPBOT;

        /**
         * If true, replay will run as fast as your computer can go and log to a log file instead of
         * NetworkTables. You will have to open the log file to see anything.
         */
        public static final boolean replayRunAsFastAsPossible = true;

        public static final boolean useNintendoSwitchProController = RobotBase.isSimulation() && System.getProperty("os.name").contains("Mac OS X");
    }
}
