package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.OperatorDashboard;
import frc.robot.OperatorDashboard.LocalReefSide;
import frc.robot.OperatorDashboard.ReefZoneSide;
import frc.robot.RobotMechanism;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.util.commands.CommandsExt;
import frc.robot.util.subsystem.SubsystemBaseExt;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.superstructure.AutoAlignLocations.*;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.createIO;

public class Superstructure extends SubsystemBaseExt {
    private final RobotState robotState = RobotState.get();
    private final RobotMechanism robotMechanism = RobotMechanism.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private final Drive drive = Drive.get();
    //    private final CoralIntake coralIntake = CoralIntake.get();
//    private final Indexer indexer = Indexer.get();
    private final Elevator elevator = Elevator.get();
    private final EndEffector endEffector = EndEffector.get();

    private final SuperstructureIO io = createIO();
    private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

    public enum Goal {
        IDLE,

        INTAKE_CORAL_WAIT_PIVOT,
        INTAKE_CORAL_INTAKING,
        INDEXING_PIVOT_DOWN,
        INDEXING_PIVOT_UP,
        HANDOFF_WAIT_ELEVATOR,
        HANDOFF_HANDING_OFF,

        SCORE_CORAL_WAIT_ELEVATOR,
        SCORE_CORAL_WAIT_CONFIRM,
        SCORE_CORAL_SCORING,

        AUTO_SCORE_CORAL_WAIT_INITIAL,
        AUTO_SCORE_CORAL_WAIT_FINAL,
        AUTO_SCORE_CORAL_SCORING,

        DESCORE_ALGAE_WAIT_ELEVATOR,
        DESCORE_ALGAE_DESCORING,

        FUNNEL_INTAKE_WAITING,
        FUNNEL_INTAKE_FINALIZING,

        EJECT,
    }

    @Getter
    private Goal goal = Goal.IDLE;

    private final AutoAlign autoAlign = new AutoAlign();

    private final Debouncer endEffectorBeamBreakDebouncerShort = new Debouncer(3 * 0.02);
    private final Debouncer endEffectorBeamBreakDebouncerLong = new Debouncer(0.15);

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

//        robotMechanism.coralIntake.rangeLigament.setColor(
//                intakeRangeTriggered()
//                        ? new Color8Bit(Color.kGreen)
//                        : new Color8Bit(Color.kRed)
//        );
//        robotMechanism.indexer.beamBreakLigament.setColor(
//                inputs.indexerBeamBreakTriggered
//                        ? new Color8Bit(Color.kGreen)
//                        : new Color8Bit(Color.kRed)
//        );
        robotMechanism.endEffector.beamBreakLigament.setColor(
                endEffectorTriggeredShort() || operatorDashboard.ignoreEndEffectorBeamBreak.get()
                        ? new Color8Bit(Color.kGreen)
                        : new Color8Bit(Color.kRed)
        );
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Goal", goal);
    }

//    private boolean intakeRangeTriggered() {
//        return inputs.intakeRangeMeters <= intakeRangeTriggerMeters;
//    }

    /** Reacts quickly to change so better for waiting for the beam break */
    @AutoLogOutput(key = "Superstructure/EndEffectorTriggeredShort")
    private boolean endEffectorTriggeredShort() {
        return endEffectorBeamBreakDebouncerShort.calculate(inputs.endEffectorBeamBreakTriggered);
    }

    /** Reacts slowly to change so better for gating commands */
    @AutoLogOutput(key = "Superstructure/EndEffectorTriggeredLong")
    private boolean endEffectorTriggeredLong() {
        return endEffectorBeamBreakDebouncerLong.calculate(inputs.endEffectorBeamBreakTriggered);
    }

//    public Command waitUntilIntakeTriggered() {
//        // This should not require the superstructure because we don't want to conflict with setGoal
//        return Commands.waitUntil(this::intakeRangeTriggered);
//    }
//
//    public Command waitUntilIndexerTriggered() {
//        // This should not require the superstructure because we don't want to conflict with setGoal
//        return Commands.waitUntil(() -> inputs.indexerBeamBreakTriggered);
//    }

    public Command waitUntilEndEffectorTriggered(Command ifIgnored) {
        // This should not require the superstructure because we don't want to conflict with setGoal
        return Commands.either(
                ifIgnored,
                Commands.waitUntil(this::endEffectorTriggeredShort),
                operatorDashboard.ignoreEndEffectorBeamBreak::get
        );
    }

    public Command waitUntilEndEffectorNotTriggered(Command ifIngored) {
        // This should not require the superstructure because we don't want to conflict with setGoal
        return Commands.either(
                ifIngored,
                Commands.waitUntil(() -> !endEffectorTriggeredShort()),
                operatorDashboard.ignoreEndEffectorBeamBreak::get
        );
    }

    public Command idle() {
        return setGoal(Goal.IDLE).andThen(Commands.idle());
    }

//    public Command coralIntakeIdle() {
//        return coralIntake.setGoals(CoralIntake.PivotGoal.STOW, CoralIntake.RollersGoal.IDLE).andThen(Commands.idle());
//    }

//    public Command indexerIdle() {
//        return indexer.setGoal(Indexer.RollersGoal.IDLE).andThen(Commands.idle());
//    }

    public Command elevatorIdle() {
        return elevator.setGoal(() -> Elevator.Goal.STOW).andThen(Commands.idle());
    }

    public Command endEffectorIdle() {
        return endEffector.setGoal(EndEffector.RollersGoal.IDLE).andThen(Commands.idle());
    }

//    public Command intakeCoral() {
//        return Commands.sequence(
//                Commands.parallel(
//                        setGoal(Goal.INTAKE_CORAL_WAIT_PIVOT),
//                        coralIntake.setGoalsAndWaitUntilAtPivotGoal(CoralIntake.PivotGoal.INTAKE, CoralIntake.RollersGoal.IDLE)
//                ),
//                Commands.parallel(
//                        setGoal(Goal.INTAKE_CORAL_INTAKING),
//                        coralIntake.setGoals(CoralIntake.PivotGoal.INTAKE, CoralIntake.RollersGoal.INTAKE),
//                        indexer.setGoal(Indexer.RollersGoal.INDEX)
//                ),
//                waitUntilIntakeTriggered(),
//                // Branch off into an uncancelable sequence to prevent indexing being messed up
//                CommandsExt.schedule(
//                        Commands.sequence(
//                                // Wait at least a small amount of time, or until we are done indexing to bring the intake up
//                                Commands.parallel(
//                                        setGoal(Goal.INDEXING_PIVOT_DOWN),
//                                        Commands.race(
//                                                Commands.waitSeconds(0.25),
//                                                waitUntilIndexerTriggered()
//                                        )
//                                ),
//                                Commands.parallel(
//                                        setGoal(Goal.INDEXING_PIVOT_UP),
//                                        coralIntake.setGoals(CoralIntake.PivotGoal.STOW, CoralIntake.RollersGoal.IDLE),
//                                        waitUntilIndexerTriggered()
//                                ),
//                                Commands.parallel(
//                                        setGoal(Goal.HANDOFF_WAIT_ELEVATOR),
//                                        indexer.setGoal(Indexer.RollersGoal.IDLE),
//                                        elevator.setGoalAndWaitUntilAtGoal(() -> Elevator.Goal.STOW)
//                                ),
//                                Commands.parallel(
//                                        setGoal(Goal.HANDOFF_HANDING_OFF),
//                                        indexer.setGoal(Indexer.RollersGoal.HANDOFF),
//                                        endEffector.setGoal(EndEffector.RollersGoal.HANDOFF),
//                                        waitUntilEndEffectorTriggered()
//                                )
//                                // TODO: move forward X radians
//                        ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
//                )
//        );
//    }

    public Command eject() {
        return Commands.parallel(
                setGoal(Goal.EJECT),
                endEffector.setGoal(EndEffector.RollersGoal.EJECT),
                Commands.idle()
        );
    }

    /** NOT FOR TELEOP USE */
    public Command scoreCoralDuringAuto(
            BooleanSupplier forwardCondition,
            Supplier<Elevator.Goal> elevatorGoalSupplier
    ) {

        return Commands.sequence(
                Commands.parallel(
                        setGoal(Goal.SCORE_CORAL_WAIT_ELEVATOR),
                        endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                        elevator.setGoalAndWaitUntilAtGoal(elevatorGoalSupplier)
                ),
                Commands.parallel(
                        setGoal(Goal.SCORE_CORAL_WAIT_CONFIRM),
                        Commands.waitUntil(forwardCondition)
                ),
                Commands.parallel(
                        setGoal(Goal.SCORE_CORAL_SCORING),
                        endEffector.setGoal(EndEffector.RollersGoal.SCORE_CORAL),
                        elevator.setGoal(elevatorGoalSupplier)
                ),
                // Wait for coral to settle
                Commands.waitSeconds(0.5)
        );
    }

    public Command scoreCoralManual(
            BooleanSupplier forwardCondition,
            BooleanSupplier cancelCondition,
            Supplier<Elevator.Goal> elevatorGoalSupplier
    ) {
        Command cmd = Commands.sequence(
                Commands.parallel(
                        setGoal(Goal.SCORE_CORAL_WAIT_ELEVATOR),
                        endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                        elevator.setGoalAndWaitUntilAtGoal(elevatorGoalSupplier)
                ),
                Commands.parallel(
                        setGoal(Goal.SCORE_CORAL_WAIT_CONFIRM),
                        Commands.waitUntil(forwardCondition)
                ),
                // Don't allow canceling
                CommandsExt.schedule(
                        Commands.sequence(
                                Commands.parallel(
                                        setGoal(Goal.SCORE_CORAL_SCORING),
                                        endEffector.setGoal(EndEffector.RollersGoal.SCORE_CORAL),
                                        elevator.setGoal(elevatorGoalSupplier),
                                        waitUntilEndEffectorNotTriggered(Commands.waitSeconds(0.5))
                                ),
                                // Wait for coral to settle
                                Commands.waitSeconds(0.75)
                        ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                )
        );
        return CommandsExt.onlyIf(
                () -> endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                CommandsExt.cancelOnTrigger(
                        cancelCondition,
                        cmd
                )
        );
    }

    public Command descoreAlgaeManual(Supplier<Elevator.Goal> elevatorGoalSupplier) {
        Command cmd = Commands.sequence(
                Commands.parallel(
                        setGoal(Goal.DESCORE_ALGAE_WAIT_ELEVATOR),
                        endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                        elevator.setGoalAndWaitUntilAtGoal(elevatorGoalSupplier)
                ),
                Commands.parallel(
                        setGoal(Goal.DESCORE_ALGAE_DESCORING),
                        endEffector.setGoal(EndEffector.RollersGoal.DESCORE_ALGAE),
                        elevator.setGoal(elevatorGoalSupplier),
                        Commands.idle()
                )
        );
        return CommandsExt.onlyIf(
                () -> !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                cmd
        );
    }

    public Command funnelIntake() {
        Command cmd = Commands.sequence(
                Commands.parallel(
                        setGoal(Goal.FUNNEL_INTAKE_WAITING),
                        endEffector.setGoal(EndEffector.RollersGoal.FUNNEL_INTAKE),
                        waitUntilEndEffectorTriggered(Commands.idle())
                ),
                // Don't allow canceling
                CommandsExt.schedule(
                        Commands.sequence(
                                Commands.parallel(
                                        setGoal(Goal.FUNNEL_INTAKE_FINALIZING),
                                        endEffector.moveByAndWaitUntilDone(Units.inchesToMeters(0.5))
                                )
                        ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                )
        );
        return CommandsExt.onlyIf(
                () -> !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                cmd
        );
    }

    private boolean isAtPoseWithTolerance(Pose2d desiredPose, double linearToleranceMeters, double angularToleranceRad) {
        Pose2d currentPose = robotState.getPose();
        return desiredPose.getTranslation().getDistance(currentPose.getTranslation()) < linearToleranceMeters
                && Math.abs(desiredPose.getRotation().minus(currentPose.getRotation()).getRadians()) < angularToleranceRad;
    }

    public Command waitUntilAtInitialPosition(Supplier<ReefZoneSide> reefSideSupplier, Supplier<LocalReefSide> sideSupplier) {
        // This should not require the superstructure because we don't want to conflict with setGoal
        return Commands.waitUntil(() -> isAtPoseWithTolerance(
                getInitialAlignPose(reefSideSupplier.get(), sideSupplier.get()),
                initialAlignToleranceMeters,
                initialAlignToleranceRad
        ));
    }

    public Command waitUntilAtFinalPosition(Supplier<ReefZoneSide> reefSideSupplier, Supplier<LocalReefSide> sideSupplier) {
        // This should not require the superstructure because we don't want to conflict with setGoal
        return Commands.waitUntil(() -> isAtPoseWithTolerance(
                        getFinalAlignPose(reefSideSupplier.get(), sideSupplier.get()),
                        finalAlignToleranceMeters,
                        finalAlignToleranceRad
                )
                        && Math.abs(drive.getMeasuredChassisLinearVelocityMetersPerSec()) < finalAlignToleranceMetersPerSecond
                        && Math.abs(drive.getMeasuredChassisAngularVelocityRadPerSec()) < finalAlignToleranceRadPerSecond
        );
    }

    /** NOT FOR TELEOP USE */
    public Command autoAlignAndScoreDuringAuto(
            ReefZoneSide reefZoneSide,
            LocalReefSide localSide,
            Elevator.Goal elevatorGoal
    ) {
        return Commands.sequence(
                // Drive to primary position and intake
                Commands.race(
                        drive.moveTo(() -> getInitialAlignPose(reefZoneSide, localSide)),
                        Commands.parallel(
                                setGoal(Goal.AUTO_SCORE_CORAL_WAIT_INITIAL),
                                endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                                elevator.setGoal(() -> Elevator.Goal.STOW),
                                waitUntilAtInitialPosition(() -> reefZoneSide, () -> localSide)
                        )
                ),
                // Drive to secondary position while raising elevator and scoring once within tolerance
                Commands.race(
                        drive.moveTo(() -> getFinalAlignPose(reefZoneSide, localSide)),
                        Commands.sequence(
                                Commands.parallel(
                                        setGoal(Goal.AUTO_SCORE_CORAL_WAIT_FINAL),
                                        endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                                        elevator.setGoalAndWaitUntilAtGoal(() -> elevatorGoal),
                                        waitUntilAtFinalPosition(() -> reefZoneSide, () -> localSide)
                                ),
                                Commands.parallel(
                                        setGoal(Goal.AUTO_SCORE_CORAL_SCORING),
                                        endEffector.setGoal(EndEffector.RollersGoal.SCORE_CORAL),
                                        waitUntilEndEffectorNotTriggered(Commands.waitSeconds(0.5))
                                ),
                                // Wait for coral to settle
                                Commands.waitSeconds(0.75)
                        )
                )
        );
    }

    public Command autoAlignAndScore(
            Supplier<ReefZoneSide> reefSideSupplier,
            Supplier<LocalReefSide> sideSupplier,
            Supplier<Elevator.Goal> elevatorGoalSupplier,
            BooleanSupplier cancelCondition
    ) {
        Command cmd = Commands.sequence(
                // Drive to primary position and intake
                Commands.race(
                        drive.moveTo(() -> getInitialAlignPose(reefSideSupplier.get(), sideSupplier.get())),
                        Commands.parallel(
                                setGoal(Goal.AUTO_SCORE_CORAL_WAIT_INITIAL),
                                endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                                elevator.setGoal(() -> Elevator.Goal.STOW),
                                waitUntilAtInitialPosition(reefSideSupplier, sideSupplier)
                        )
                ),
                // Drive to secondary position while raising elevator
                Commands.race(
                        drive.moveTo(() -> getFinalAlignPose(reefSideSupplier.get(), sideSupplier.get())),
                        Commands.parallel(
                                setGoal(Goal.AUTO_SCORE_CORAL_WAIT_FINAL),
                                endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                                elevator.setGoalAndWaitUntilAtGoal(elevatorGoalSupplier),
                                waitUntilAtFinalPosition(reefSideSupplier, sideSupplier)
                        )
                ),
                // don't allow cancelling
                CommandsExt.schedule(
                        Commands.race(
                                drive.moveTo(() -> getFinalAlignPose(reefSideSupplier.get(), sideSupplier.get())),
                                Commands.sequence(
                                        Commands.parallel(
                                                setGoal(Goal.AUTO_SCORE_CORAL_SCORING),
                                                endEffector.setGoal(EndEffector.RollersGoal.SCORE_CORAL),
                                                waitUntilEndEffectorNotTriggered(Commands.waitSeconds(0.5))
                                        ),
                                        // Wait for coral to settle
                                        Commands.waitSeconds(0.75)
                                )
                        ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                )
        );
        return CommandsExt.onlyIf(
                // Only run if you have coral and are in front of your reef side
                () -> (endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get())
                        && alignable(reefSideSupplier.get(), RobotState.get().getPose()),
                CommandsExt.cancelOnTrigger(
                        cancelCondition,
                        cmd
                )
        );
    }
}