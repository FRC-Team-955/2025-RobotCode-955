package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.RobotMechanism;
import frc.robot.RobotState;
import frc.robot.subsystems.coralintake.CoralIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.util.commands.CommandsExt;
import frc.robot.util.subsystem.SubsystemBaseExt;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.intakeRangeTriggerMeters;

public class Superstructure extends SubsystemBaseExt {
    private final RobotState robotState = RobotState.get();
    private final RobotMechanism robotMechanism = robotState.getMechanism();
    private final Drive drive = Drive.get();
    private final CoralIntake coralIntake = CoralIntake.get();
    private final Indexer indexer = Indexer.get();
    private final Elevator elevator = Elevator.get();
    private final EndEffector endEffector = EndEffector.get();

    public enum Goal {
        IDLE,

        INTAKE_CORAL_WAIT_PIVOT,
        INTAKE_CORAL_INTAKING,
        INTAKE_CORAL_INDEXING_PIVOT_DOWN,
        INTAKE_CORAL_INDEXING_PIVOT_UP,
        INTAKE_CORAL_DONE,

        SCORE_CORAL
    }

    private final SuperstructureIO io = SuperstructureConstants.io;
    private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

    @Getter
    private Goal goal = Goal.IDLE;

    private Command withGoal(Goal goal, Command command) {
        return new WrapperCommand(command) {
            @Override
            public void initialize() {
                Superstructure.this.goal = goal;
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
        // TODO: alerts for everything - not just superstructure

        robotMechanism.coralIntake.rangeLigament.setColor(
                inputs.intakeRangeMeters <= intakeRangeTriggerMeters
                        ? new Color8Bit(Color.kGreen)
                        : new Color8Bit(Color.kRed)
        );
        robotMechanism.indexer.beamBreakLigament.setColor(
                inputs.indexerBeamBreakTriggered
                        ? new Color8Bit(Color.kGreen)
                        : new Color8Bit(Color.kRed)
        );
        robotMechanism.endEffector.beamBreakLigament.setColor(
                inputs.endEffectorBeamBreakTriggered
                        ? new Color8Bit(Color.kGreen)
                        : new Color8Bit(Color.kRed)
        );
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Goal", goal);
    }

    public Command waitUntilIntakeTriggered() {
        // This should not require the superstructure because we don't want to conflict with setGoal
        return Commands.waitUntil(() -> inputs.intakeRangeMeters <= intakeRangeTriggerMeters);
    }

    public Command waitUntilIndexerTriggered() {
        // This should not require the superstructure because we don't want to conflict with setGoal
        return Commands.waitUntil(() -> inputs.indexerBeamBreakTriggered);
    }

    public Command waitUntilEndEffectorTriggered() {
        // This should not require the superstructure because we don't want to conflict with setGoal
        return Commands.waitUntil(() -> inputs.endEffectorBeamBreakTriggered);
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
                        indexer.setGoal(Indexer.RollersGoal.INDEX)
                ),
                waitUntilIntakeTriggered(),
                // Branch off into an uncancelable sequence to prevent indexing being messed up
                CommandsExt.schedule(
                        Commands.sequence(
                                // Wait at least a small amount of time, or until we are done indexing to bring the intake up
                                Commands.parallel(
                                        setGoal(Goal.INTAKE_CORAL_INDEXING_PIVOT_DOWN),
                                        Commands.race(
                                                Commands.waitSeconds(0.25),
                                                waitUntilIndexerTriggered()
                                        )
                                ),
                                Commands.parallel(
                                        setGoal(Goal.INTAKE_CORAL_INDEXING_PIVOT_UP),
                                        coralIntake.setGoals(CoralIntake.PivotGoal.STOW, CoralIntake.RollersGoal.IDLE),
                                        waitUntilIndexerTriggered()
                                ),
                                Commands.parallel(
                                        // Log that we finished for one loop cycle
                                        // Will be cleared the next cycle since default command kicks in
                                        setGoal(Goal.INTAKE_CORAL_DONE),
                                        indexer.setGoal(Indexer.RollersGoal.IDLE)
                                )
                        ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                )
        );
    }
}