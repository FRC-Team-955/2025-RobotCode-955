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

    public static final Mode mode = RobotBase.isReal()
            ? Mode.REAL
            : (Simulation.shouldReplay ? Mode.REPLAY : Mode.SIM);

    public static final boolean isReplay = mode == Mode.REPLAY;

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
        /**
         * Set this to a RobotIdentity to replay using a log from that robot.
         * If replayIdentity is null, log replay will not run and simulation will run like normal.
         */
        public static final RobotIdentity replayIdentity = null;
        public static final boolean shouldReplay = RobotBase.isSimulation() && replayIdentity != null; // Don't modify please!

        /**
         * If true, replay will run as fast as your computer can go and log to a log file instead of
         * NetworkTables. You will have to open the log file to see anything.
         */
        public static final boolean replayRunAsFastAsPossible = true;

        public static final boolean useNintendoSwitchProController = RobotBase.isSimulation() && System.getProperty("os.name").contains("Mac OS X");
    }
}
