package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldLocations;
import frc.robot.OperatorDashboard;
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

import java.util.function.Supplier;

import static frc.robot.FieldLocations.*;
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

        DESCORE_ALGAE_WAIT_ELEVATOR,
        DESCORE_ALGAE_DESCORING,

        FUNNEL_INTAKE_WAITING,
        FUNNEL_INTAKE_FINALIZING,

        EJECT,
    }

    @Getter
    private Goal goal = Goal.IDLE;

    private final SuperstructureIO io = createIO();
    private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

    private final Debouncer endEffectorBeamBreakDebouncer = new Debouncer(0.05);

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
                endEffectorTriggered() || operatorDashboard.ignoreEndEffectorBeamBreak.get()
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

//    private boolean intakeRangeTriggered() {
//        return inputs.intakeRangeMeters <= intakeRangeTriggerMeters;
//    }

    @AutoLogOutput(key = "Superstructure/EndEffectorTriggered")
    private boolean endEffectorTriggered() {
        return endEffectorBeamBreakDebouncer.calculate(inputs.endEffectorBeamBreakTriggered);
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
                Commands.waitUntil(this::endEffectorTriggered),
                operatorDashboard.ignoreEndEffectorBeamBreak::get
        );
    }

    public Command waitUntilEndEffectorNotTriggered(Command ifIngored) {
        // This should not require the superstructure because we don't want to conflict with setGoal
        return Commands.either(
                ifIngored,
                Commands.waitUntil(() -> !endEffectorTriggered()),
                operatorDashboard.ignoreEndEffectorBeamBreak::get
        );
    }

    public Command waitUntilAtPrimaryPosition(Supplier<Integer> reefSideSupplier) {
        // This should not require the superstructure because we don't want to conflict with setGoal
        return Commands.waitUntil(
                () -> getDistance(robotState.getPose(), getPrimaryAlignPose(reefSideSupplier.get())) < primaryAlignToleranceMeters
                        && Math.abs(getAngle(robotState.getPose(), getPrimaryAlignPose(reefSideSupplier.get())).getRadians()) < primaryAlignToleranceRad
        );
    }

    public Command waitUntilAtSecondaryPosition(Supplier<Integer> reefSideSupplier, Supplier<Boolean> alignLeftSupplier) {
        // This should not require the superstructure because we don't want to conflict with setGoal
        return Commands.waitUntil(
                () -> getDistance(robotState.getPose(), getSecondaryAlignPose(reefSideSupplier.get(), alignLeftSupplier.get())) < secondaryAlignToleranceMeters
                        && Math.abs(getAngle(robotState.getPose(), getSecondaryAlignPose(reefSideSupplier.get(), alignLeftSupplier.get())).getRadians()) < secondaryAlignToleranceRad
        );
    }

    public Command waitUntilAtFinalPosition(Supplier<Integer> reefSideSupplier, Supplier<Boolean> alignLeftSupplier) {
        // This should not require the superstructure because we don't want to conflict with setGoal
        return Commands.waitUntil(
                () -> getDistance(robotState.getPose(), getFinalAlignPose(reefSideSupplier.get(), alignLeftSupplier.get())) < finalAlignToleranceMeters
                        && Math.abs(getAngle(robotState.getPose(), getFinalAlignPose(reefSideSupplier.get(), alignLeftSupplier.get())).getRadians()) < finalAlignToleranceRad
                        && Math.abs(drive.getMeasuredChassisLinearVelocityMetersPerSec()) < finalAlignToleranceMetersPerSecond
                        && Math.abs(drive.getMeasuredChassisAngularVelocityRadPerSec()) < finalAlignToleranceRadPerSecond
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
            Trigger forwardTrigger,
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
                        Commands.waitUntil(forwardTrigger)
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
            Trigger forwardTrigger,
            Trigger cancelTrigger,
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
                        Commands.waitUntil(forwardTrigger)
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
                () -> endEffectorTriggered() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                CommandsExt.cancelOnTrigger(
                        cancelTrigger,
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
                () -> !endEffectorTriggered() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
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
                                        endEffector.setGoal(EndEffector.RollersGoal.ORIENT_CORAL),
                                        Commands.waitSeconds(0.20)
                                )
                        ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                )
        );
        return CommandsExt.onlyIf(
                () -> !endEffectorTriggered() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                cmd
        );
    }

    public Command autoAlignAndScore(
            Supplier<Integer> reefSideSupplier,
            Supplier<Boolean> alignLeftSupplier,
            Supplier<Elevator.Goal> elevatorGoalSupplier,
            Trigger cancelTrigger
    ) {
        Command cmd = Commands.sequence(
                // Drive to primary position and intake
                Commands.race(
                        drive.moveTo(() -> getPrimaryAlignPose(reefSideSupplier.get())),
                        waitUntilAtPrimaryPosition(reefSideSupplier)
                ),
                // Drive to secondary position while raising elevator
                Commands.race(
                        drive.moveTo(() -> getSecondaryAlignPose(reefSideSupplier.get(), alignLeftSupplier.get())),
                        Commands.parallel(
                                setGoal(Goal.SCORE_CORAL_WAIT_ELEVATOR),
                                endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                                elevator.setGoalAndWaitUntilAtGoal(elevatorGoalSupplier),
                                waitUntilAtSecondaryPosition(reefSideSupplier, alignLeftSupplier)
                        )
                ),
                // Drive to final position and score
                Commands.race(
                        drive.moveTo(() -> getFinalAlignPose(reefSideSupplier.get(), alignLeftSupplier.get())),
                        waitUntilAtFinalPosition(reefSideSupplier, alignLeftSupplier)
                ),
                // don't allow cancelling
                CommandsExt.schedule(
                        Commands.race(
                                drive.moveTo(() -> getFinalAlignPose(reefSideSupplier.get(), alignLeftSupplier.get())),
                                Commands.sequence(
                                        Commands.parallel(
                                                setGoal(Goal.SCORE_CORAL_SCORING),
                                                endEffector.setGoal(EndEffector.RollersGoal.SCORE_CORAL),
                                                waitUntilEndEffectorNotTriggered(Commands.waitSeconds(0.5))
                                        ),
                                        // Wait for coral to settle
                                        Commands.waitSeconds(0.75)
                                ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                        )
                )
        );
        return CommandsExt.onlyIf(
                // Only run if you have coral and are in front of your reef side
                () -> (endEffectorTriggered() || operatorDashboard.ignoreEndEffectorBeamBreak.get())
                        && alignable(reefSideSupplier.get(), RobotState.get().getPose()),
                CommandsExt.cancelOnTrigger(
                        cancelTrigger,
                        cmd
                )
        );
    }
}