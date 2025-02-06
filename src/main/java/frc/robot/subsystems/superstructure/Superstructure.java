package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.coralintake.CoralIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.util.SubsystemBaseExt;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

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

        INTAKE_CORAL_WAIT_PIVOT,
        INTAKE_CORAL_INTAKING,
        
        SCORE_CORAL;
    }

    private final SuperstructureIO io = SuperstructureConstants.io;
    private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

    @Getter
    private Goal goal = Goal.IDLE;

    private Command withGoal(Goal goal, Command command) {
        return new WrapperCommand(command) {
            @Override
            public void initialize() {
                this.goal = goal;
                super.initialize();
            }
        };
    }

    private Command setGoal(Goal goal) {
        return runOnce(() -> this.goal = goal);
    }

    private static Superstructure instance;

    public static Superstructure get() {
        if (instance == null)
            synchronized (Superstructure.class) {
                instance = new Superstructure();
            }

        return instance;
    }

    private Superstructure() {
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superstructure", inputs);
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Goal", goal);
    }

    public Command waitUntilIntakeTriggered() {
        return waitUntil(() -> inputs.intakeRangeMeters <= inputs.intakeRangeConnected);
    }

    public Command waitUntilIndexerTriggered() {
        return waitUntil(() -> inputs.indexerBeamBreakTriggered);
    }

    public Command waitUntilEndEffectorTriggered() {
        return waitUntil(() -> inputs.endEffectorBeamBreakTriggered);
    }

    public Command idle() {
        return setGoal(Goal.IDLE).andThen(Commands.idle());
    }

    public Command coralIntakeIdle() {
        return coralIntake.setGoals(CoralIntake.PivotGoal.STOW, CoralIntake.RollersGoal.IDLE).andThen(Commands.idle());
    }

    public Command indexerIdle() {
        return indexer.setGoal(Indexer.RollersGoal.IDLE).andThen(Commands.idle());
    }

    public Command elevatorIdle() {
        return elevator.setGoal(Elevator.Goal.STOW).andThen(Commands.idle());
    }

    public Command endEffectorIdle() {
        return endEffector.setGoal(EndEffector.RollersGoal.IDLE).andThen(Commands.idle());
    }

    public Command coralIntake() {
        return Commands.sequence(
            Commands.parallel(
                setGoal(Goal.INTAKE_CORAL_WAIT_PIVOT),
                coralIntake.setGoalsAndWaitUntilAtPivotGoal(CoralIntake.PivotGoal.INTAKE, CoralIntake.RollersGoal.IDLE)
            ),
            Commands.parallel(
                setGoal(Goal.INTAKE_CORAL_INTAKING),
                coralIntake.setGoals(CoralIntake.PivotGoal.INTAKE, CoralIntake.RollersGoal.INTAKE),
                indexer.setGoals(Indexer.RollersGoal.INDEX)
            ),
            waitUntilIntakeTriggered(),
            // Branch off into an uncancelable sequence to prevent indexing being messed up
            // Proxy won't work (?) because the outer seqence could be cancellable?
            // TODO: test this part
            CommandsExt.schedule(Commands.sequence(
                Commands.race(
                    Commands.waitSeconds(1),
                    waitUntilIndexerTriggered()
                ),
                Commands.parallel(
                    coralIntake.setGoals(CoralIntake.PivotGoal.STOW, CoralIntake.RollersGoal.IDLE),
                    waitUntilIndexerTriggered()
                ),
                indexer.setGoals(Indexer.RollersGoal.IDLE)
            ).withInterruptBehavior(InterruptionBehavor.kCancelIncoming))
        );
    }
}