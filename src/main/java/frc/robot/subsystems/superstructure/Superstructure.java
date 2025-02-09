package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.*;
import frc.robot.subsystems.coralintake.CoralIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.util.commands.CommandsExt;
import frc.robot.util.subsystem.SubsystemBaseExt;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static frc.robot.FieldLocations.*;
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
        INDEXING_PIVOT_DOWN,
        INDEXING_PIVOT_UP,
        HANDOFF_WAIT_ELEVATOR,
        HANDOFF_HANDING_OFF,

        SCORE_CORAL_WAIT_ELEVATOR,
        SCORE_CORAL_SCORING,
        SCORE_CORAL_DONE
    }

    @Getter
    private Goal goal = Goal.IDLE;

    private final SuperstructureIO io = SuperstructureConstants.io;
    private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

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
                intakeRangeTriggered()
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
//        Logger.recordOutput("Superstructure/AprilTagPose6", FieldLocations.getAprilTagPose(6));
//        Logger.recordOutput("Superstructure/AprilTagPoseAdjusted", FieldLocations.getAprilTagPoseAdjusted(0));
//        Logger.recordOutput("Superstructure/PrimaryAlignPose", FieldLocations.getPrimaryAlignPose(0));
        Logger.recordOutput("Superstructure/AtPrimaryPose", FieldLocations.getDistance(robotState.getPose(), FieldLocations.getPrimaryAlignPose(0)));
        Logger.recordOutput("Superstructure/AtSecondaryPose", getDistance(robotState.getPose(), getSecondaryAlignPose(0, true)) < secondaryAlignToleranceMeters
                && Math.abs(getAngle(robotState.getPose(), getSecondaryAlignPose(0, true)).getRadians()) < secondaryAlignToleranceRad
        );
//        Logger.recordOutput("Superstructure/SecondaryAlignPose", FieldLocations.getSecondaryAlignPose(0, true));
//        Logger.recordOutput("Superstructure/FinalAlignPose", FieldLocations.getFinalAlignPose(0, true));
        Logger.recordOutput("Superstructure/Alignable", FieldLocations.alignable(0, robotState.getPose()));
    }

    private boolean intakeRangeTriggered() {
        return inputs.intakeRangeMeters <= intakeRangeTriggerMeters;
    }

    public Command waitUntilIntakeTriggered() {
        // This should not require the superstructure because we don't want to conflict with setGoal
        return Commands.waitUntil(this::intakeRangeTriggered);
    }

    public Command waitUntilIndexerTriggered() {
        // This should not require the superstructure because we don't want to conflict with setGoal
        return Commands.waitUntil(() -> inputs.indexerBeamBreakTriggered);
    }

    public Command waitUntilEndEffectorTriggered() {
        // This should not require the superstructure because we don't want to conflict with setGoal
        return Commands.waitUntil(() -> inputs.endEffectorBeamBreakTriggered);
    }

    public Command waitUntilEndEffectorNotTriggered() {
        // This should not require the superstructure because we don't want to conflict with setGoal
        return Commands.waitUntil(() -> !inputs.endEffectorBeamBreakTriggered);
    }

    public Command waitUntilAtPrimaryPosition(int reefSide) {
        // This should not require the superstructure because we don't want to conflict with setGoal
        return Commands.waitUntil(
                () -> getDistance(robotState.getPose(), getPrimaryAlignPose(reefSide)) < primaryAlignToleranceMeters
                    && Math.abs(getAngle(robotState.getPose(), getPrimaryAlignPose(reefSide)).getRadians()) < primaryAlignToleranceRad
        );
    }

    public Command waitUntilAtSecondaryPosition(int reefSide, boolean alignLeft) {
        // This should not require the superstructure because we don't want to conflict with setGoal
        return Commands.waitUntil(
                () -> getDistance(robotState.getPose(), getSecondaryAlignPose(reefSide, alignLeft)) < secondaryAlignToleranceMeters
                        && Math.abs(getAngle(robotState.getPose(), getSecondaryAlignPose(reefSide, alignLeft)).getRadians()) < secondaryAlignToleranceRad
        );
    }

    public Command waitUntilAtFinalPosition(int reefSide, boolean alignLeft) {
        // This should not require the superstructure because we don't want to conflict with setGoal
        return Commands.waitUntil(
                () -> getDistance(robotState.getPose(), getFinalAlignPose(reefSide, alignLeft)) < finalAlignToleranceMeters
                        && Math.abs(getAngle(robotState.getPose(), getFinalAlignPose(reefSide, alignLeft)).getRadians()) < finalAlignToleranceRad
        );
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

    public Command intakeCoral() {
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
                                        setGoal(Goal.INDEXING_PIVOT_DOWN),
                                        Commands.race(
                                                Commands.waitSeconds(0.25),
                                                waitUntilIndexerTriggered()
                                        )
                                ),
                                Commands.parallel(
                                        setGoal(Goal.INDEXING_PIVOT_UP),
                                        coralIntake.setGoals(CoralIntake.PivotGoal.STOW, CoralIntake.RollersGoal.IDLE),
                                        waitUntilIndexerTriggered()
                                ),
                                Commands.parallel(
                                        setGoal(Goal.HANDOFF_WAIT_ELEVATOR),
                                        indexer.setGoal(Indexer.RollersGoal.IDLE),
                                        elevator.setGoalAndWaitUntilAtGoal(Elevator.Goal.STOW)
                                ),
                                Commands.parallel(
                                        setGoal(Goal.HANDOFF_HANDING_OFF),
                                        indexer.setGoal(Indexer.RollersGoal.HANDOFF),
                                        endEffector.setGoal(EndEffector.RollersGoal.HANDOFF),
                                        waitUntilEndEffectorTriggered()
                                )
                                // TODO: move forward X radians
                        ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                )
        );
    }

    public Command scoreCoralManual(Trigger forwardTrigger, Trigger cancelTrigger) {
        return CommandsExt.onlyIf(
                () -> inputs.endEffectorBeamBreakTriggered,
                CommandsExt.cancelOnTrigger(
                        cancelTrigger,
                        Commands.sequence(
                                Commands.parallel(
                                        setGoal(Goal.SCORE_CORAL_WAIT_ELEVATOR),
                                        endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                                        elevator.setGoalAndWaitUntilAtGoal(Elevator.Goal.SCORE_L4)
                                ),
                                Commands.waitUntil(forwardTrigger),
                                // Don't allow canceling
                                CommandsExt.schedule(
                                        Commands.sequence(
                                                Commands.parallel(
                                                        setGoal(Goal.SCORE_CORAL_SCORING),
                                                        endEffector.setGoal(EndEffector.RollersGoal.SCORE),
                                                        waitUntilEndEffectorNotTriggered()
                                                ),
                                                // Wait for coral to settle
                                                Commands.waitSeconds(0.75)
                                        ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                                )
                        )
                )
        );
    }

    public Command autoAlignAndScore(int reefSide, boolean alignLeft, Trigger cancelTrigger) {
        return CommandsExt.onlyIf(
                // Only run if you have coral and are in front of your reef side
                () -> (inputs.endEffectorBeamBreakTriggered)
                        && alignable(reefSide, RobotState.get().getPose()),
                CommandsExt.cancelOnTrigger(
                        cancelTrigger,
                        Commands.sequence(
                                // Drive to primary position and intake
                                Commands.race(
                                        drive.moveTo(() -> getPrimaryAlignPose(reefSide)),
                                        waitUntilAtPrimaryPosition(reefSide)
                                ),
                                // Drive to secondary position while raising elevator
                                Commands.race(
                                        drive.moveTo(() -> getSecondaryAlignPose(reefSide, alignLeft)),
                                        Commands.parallel(
                                                setGoal(Goal.SCORE_CORAL_WAIT_ELEVATOR),
                                                endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                                                elevator.setGoalAndWaitUntilAtGoal(Elevator.Goal.SCORE_L4),
                                                waitUntilAtSecondaryPosition(reefSide, alignLeft)
                                        )
                                ),
                                // Drive to final position and score
                                Commands.race(
                                        drive.moveTo(() -> getFinalAlignPose(reefSide, alignLeft)),
                                        waitUntilAtFinalPosition(reefSide, alignLeft)
                                ),
                                // don't allow cancelling
                                CommandsExt.schedule(
                                        Commands.race(
                                                drive.moveTo(() -> getFinalAlignPose(reefSide, alignLeft)),
                                                Commands.sequence(
                                                        Commands.parallel(
                                                                setGoal(Goal.SCORE_CORAL_SCORING),
                                                                endEffector.setGoal(EndEffector.RollersGoal.SCORE),
                                                                waitUntilEndEffectorNotTriggered()
                                                        ),
                                                        // Wait for coral to settle
                                                        Commands.waitSeconds(0.75)
                                                ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                                        )
                                )
                        )
                )
        );
    }
}