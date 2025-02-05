package frc.robot.subsystems.superstructure;

public class Superstructure extends SubsystemBaseExt {
    private final RobotState robotState = RobotState.get();
    private final Drive drive = Drive.get();
    private final CoralIntake coralIntake = CoralIntake.get();
    private final Indexer indexer = Indexer.get();
    private final Elevator elevator = Elevator.get();
    private final EndEffector endEffector = EndEffector.get();

    public enum Goal {
        CHARACTERIZATION,
        IDLE,
        INTAKE_CORAL,
        SCORE_CORAL_L1,
        SCORE_CORAL_L2,
        SCORE_CORAL_L3,
        SCORE_CORAL_L4
    }

    private final SuperstructureIO io = SuperstructureConstants.io;
    private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

    private static Superstructure instance;

    public static Superstructure get() {
        if (instance == null)
            synchronized (Superstructure.class) {
                instance = new Superstructure();
            }

        return instance;
    }
}