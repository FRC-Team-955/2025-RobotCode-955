package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.util.commands.CommandsExt;
import frc.robot.util.subsystem.SubsystemBaseExt;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.superstructure.AutoAlignLocations.*;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.createIO;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.scoreCoralSettleSeconds;
import static frc.robot.subsystems.superstructure.SuperstructureTuning.funnelIntakeFinalizeInches;

public class Superstructure extends SubsystemBaseExt {
    private final RobotState robotState = RobotState.get();
    private final RobotMechanism robotMechanism = RobotMechanism.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private final Drive drive = Drive.get();
    private final Elevator elevator = Elevator.get();
    private final EndEffector endEffector = EndEffector.get();
    private final Funnel funnel = Funnel.get();

    private final SuperstructureIO io = createIO();
    private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

    public enum Goal {
        IDLE,

        MANUAL_SCORE_CORAL_WAIT_ELEVATOR,
        MANUAL_SCORE_CORAL_WAIT_CONFIRM,
        MANUAL_SCORE_CORAL_SCORING,

        AUTO_SCORE_CORAL_WAIT_INITIAL,
        AUTO_SCORE_CORAL_WAIT_FINAL,
        AUTO_SCORE_CORAL_WAIT_ELEVATOR,
        AUTO_SCORE_CORAL_SCORING,

        DESCORE_ALGAE_WAIT_ELEVATOR,
        DESCORE_ALGAE_DESCORING,

        FUNNEL_INTAKE_WAITING,
        FUNNEL_INTAKE_FINALIZING,

        AUTO_FUNNEL_INTAKE_WAITING_ALIGN,
        AUTO_FUNNEL_INTAKE_WAITING_SHAKE,
        AUTO_FUNNEL_INTAKE_FINALIZING,

        EJECT,
    }

    @Getter
    private Goal goal = Goal.IDLE;

    private final Debouncer endEffectorBeamBreakDebouncerShort = new Debouncer(3 * 0.02);
    private final Debouncer endEffectorBeamBreakDebouncerLong = new Debouncer(0.25);

    private final Debouncer funnelBeamBreakDebouncerShort = new Debouncer(3 * 0.02);
    private final Debouncer funnelBeamBreakDebouncerLong = new Debouncer(0.25);

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

        robotMechanism.funnel.beamBreakLigament.setColor(
                funnelTriggeredLong()
                        ? new Color8Bit(Color.kGreen)
                        : new Color8Bit(Color.kRed)
        );
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

    /** Reacts quickly to change so better for waiting for the beam break */
    @AutoLogOutput(key = "Superstructure/EndEffectorTriggeredShort")
    public boolean endEffectorTriggeredShort() {
        return endEffectorBeamBreakDebouncerShort.calculate(inputs.endEffectorBeamBreakTriggered);
    }

    /** Reacts slowly to change so better for gating commands */
    @AutoLogOutput(key = "Superstructure/EndEffectorTriggeredLong")
    public boolean endEffectorTriggeredLong() {
        return endEffectorBeamBreakDebouncerLong.calculate(inputs.endEffectorBeamBreakTriggered);
    }

    /** Reacts quickly to change so better for waiting for the beam break */
    @AutoLogOutput(key = "Superstructure/FunnelTriggeredShort")
    public boolean funnelTriggeredShort() {
        return funnelBeamBreakDebouncerShort.calculate(inputs.funnelBeamBreakTriggered);
    }

    /** Reacts slowly to change so better for gating commands */
    @AutoLogOutput(key = "Superstructure/FunnelTriggeredLong")
    public boolean funnelTriggeredLong() {
        return funnelBeamBreakDebouncerLong.calculate(inputs.funnelBeamBreakTriggered);
    }

    public Command waitUntilEndEffectorTriggered(Command ifIgnored) {
        return Commands.either(
                ifIgnored,
                Commands.waitUntil(this::endEffectorTriggeredShort),
                operatorDashboard.ignoreEndEffectorBeamBreak::get
        );
    }

    public Command waitUntilFunnelTriggered() {
        return Commands.waitUntil(this::funnelTriggeredShort);
    }

    public Command waitUntilEndEffectorNotTriggered(Command ifIngored) {
        return Commands.either(
                ifIngored,
                Commands.waitUntil(() -> !endEffectorTriggeredShort()),
                operatorDashboard.ignoreEndEffectorBeamBreak::get
        );
    }

    public Command idle() {
        return setGoal(Goal.IDLE).andThen(Commands.idle());
    }

    public Command elevatorIdle() {
        return elevator.setGoal(() -> Elevator.Goal.STOW).andThen(Commands.idle());
    }

    public Command endEffectorIdle() {
        return endEffector.setGoal(EndEffector.RollersGoal.IDLE).andThen(Commands.idle());
    }

    public Command funnelIdle() {
        return funnel.setGoal(Funnel.Goal.IDLE).andThen(Commands.idle());
    }

    public Command eject() {
        return Commands.parallel(
                setGoal(Goal.EJECT),
                endEffector.setGoal(EndEffector.RollersGoal.EJECT),
                Commands.idle()
        );
    }

    public Command scoreCoralManual(
            boolean duringAuto,
            BooleanSupplier forwardCondition,
            BooleanSupplier cancelCondition,
            Supplier<Elevator.Goal> elevatorGoalSupplier
    ) {
        Command raiseElevator = Commands.parallel(
                setGoal(Goal.MANUAL_SCORE_CORAL_WAIT_ELEVATOR),
                endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                elevator.setGoalAndWaitUntilAtGoal(elevatorGoalSupplier)
        );
        Command waitConfirm = Commands.parallel(
                setGoal(Goal.MANUAL_SCORE_CORAL_WAIT_CONFIRM),
                Commands.waitUntil(forwardCondition)
        );
        Command score = Commands.parallel(
                setGoal(Goal.MANUAL_SCORE_CORAL_SCORING),
                endEffector.setGoal(EndEffector.RollersGoal.SCORE_CORAL),
                elevator.setGoal(elevatorGoalSupplier),
                duringAuto
                        ? Commands.none()
                        : waitUntilEndEffectorNotTriggered(Commands.waitSeconds(0.5))
        );
        // Wait for coral to settle
        Command finalize = Commands.waitSeconds(scoreCoralSettleSeconds);
        if (duringAuto) {
            return Commands.sequence(raiseElevator, waitConfirm, score, finalize);
        } else {
            Command cmd = Commands.sequence(
                    raiseElevator,
                    waitConfirm,
                    // Don't allow canceling
                    CommandsExt.schedule(score.andThen(finalize).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming))
            );
            return CommandsExt.onlyIf(
                    () -> endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                    CommandsExt.cancelOnTrigger(
                            cancelCondition,
                            cmd
                    )
            );
        }
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

    public Command funnelIntake(boolean duringAuto) {
        Command intake = Commands.race(
                waitUntilEndEffectorTriggered(Commands.idle()),
                waitUntilFunnelTriggered()
        ).deadlineFor(
                setGoal(Goal.FUNNEL_INTAKE_WAITING),
                endEffector.setGoal(EndEffector.RollersGoal.FUNNEL_INTAKE),
                funnel.setGoal(Funnel.Goal.INTAKE)
        );
        Command finalize = Commands.sequence(
                waitUntilEndEffectorTriggered(Commands.idle()).deadlineFor(
                        setGoal(Goal.FUNNEL_INTAKE_FINALIZING),
                        endEffector.setGoal(EndEffector.RollersGoal.FUNNEL_INTAKE),
                        funnel.setGoal(Funnel.Goal.INTAKE)
                ),
                Commands.parallel(
                        setGoal(Goal.FUNNEL_INTAKE_FINALIZING),
                        endEffector.moveByAndWaitUntilDone(() -> Units.inchesToMeters(funnelIntakeFinalizeInches.get())),
                        funnel.setGoal(Funnel.Goal.IDLE)
                )
        );
        if (duringAuto) {
            return CommandsExt.onlyIf(
                    () -> !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                    Commands.sequence(intake, finalize)
            );
        } else {
            return CommandsExt.onlyIf(
                    () -> !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                    Commands.sequence(
                            intake,
                            // Don't allow canceling
                            CommandsExt.schedule(finalize.withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming))
                    )
            );
        }
    }

    public Command funnelIntakeWithAutoAlign(boolean duringAuto, Station station) {
        Supplier<Pose2d> alignPoseSupplier = () -> getStationAlignPose(station);
        Command intake = Commands.race(
                waitUntilEndEffectorTriggered(Commands.idle()),
                waitUntilFunnelTriggered()
        ).deadlineFor(
                endEffector.setGoal(EndEffector.RollersGoal.FUNNEL_INTAKE),
                funnel.setGoal(Funnel.Goal.INTAKE),
                Commands.sequence(
                        Commands.parallel(
                                setGoal(Goal.AUTO_FUNNEL_INTAKE_WAITING_ALIGN),
                                drive.moveTo(alignPoseSupplier).until(() -> isAtPoseWithTolerance(
                                        alignPoseSupplier.get(),
                                        stationAlignToleranceXYMeters,
                                        stationAlignToleranceOmegaRad
                                ))
                        ),
                        Commands.parallel(
                                setGoal(Goal.AUTO_FUNNEL_INTAKE_WAITING_SHAKE),
                                drive.runRobotRelative(() -> Timer.getTimestamp() % 0.25 < 0.125
                                        ? new ChassisSpeeds(-0.1, -0.1, -0.2)
                                        : new ChassisSpeeds(0.1, 0.1, 0.2))
                        )
                )
        );
        Command finalize = Commands.sequence(
                waitUntilEndEffectorTriggered(Commands.idle()).deadlineFor(
                        setGoal(Goal.AUTO_FUNNEL_INTAKE_FINALIZING),
                        endEffector.setGoal(EndEffector.RollersGoal.FUNNEL_INTAKE),
                        funnel.setGoal(Funnel.Goal.INTAKE)
                ),
                Commands.parallel(
                        endEffector.moveByAndWaitUntilDone(() -> Units.inchesToMeters(funnelIntakeFinalizeInches.get())),
                        funnel.setGoal(Funnel.Goal.IDLE)
                )
        );
        if (duringAuto) {
            return CommandsExt.onlyIf(
                    () -> !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                    Commands.sequence(intake, finalize)
            );
        } else {
            return CommandsExt.onlyIf(
                    () -> !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                    Commands.sequence(
                            intake,
                            // Don't allow canceling
                            CommandsExt.schedule(finalize.withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming))
                    )
            );
        }
    }

    private boolean isAtPoseWithTolerance(Pose2d desiredPose, double linearToleranceMeters, double angularToleranceRad) {
        Pose2d currentPose = robotState.getPose();
        return desiredPose.getTranslation().getDistance(currentPose.getTranslation()) < linearToleranceMeters
                && Math.abs(desiredPose.getRotation().minus(currentPose.getRotation()).getRadians()) < angularToleranceRad;
    }

    private boolean isAtPoseWithTolerance(Pose2d desiredPose, double toleranceXMeters, double toleranceYMeters, double angularToleranceRad) {
        Pose2d currentPose = robotState.getPose();
        Transform2d relative = new Transform2d(desiredPose, currentPose);
        return Math.abs(relative.getX()) < toleranceXMeters
                && Math.abs(relative.getY()) < toleranceYMeters
                && Math.abs(relative.getRotation().getRadians()) < angularToleranceRad;
    }

    @Getter
    private boolean autoScoreForceable = false;

    public Command autoAlignAndScore(
            boolean duringAuto,
            Supplier<ReefZoneSide> reefSideSupplier,
            Supplier<LocalReefSide> sideSupplier,
            Supplier<Elevator.Goal> elevatorGoalSupplier,
            BooleanSupplier forceCondition,
            BooleanSupplier cancelCondition
    ) {
        Supplier<Pose2d> initialPoseSupplier = () -> getInitialAlignPose(robotState.getPose(), reefSideSupplier.get(), sideSupplier.get());
        Command initial = Commands.race(
                // Drive to initial position
                drive.moveTo(initialPoseSupplier),
                Commands.parallel(
                        setGoal(Goal.AUTO_SCORE_CORAL_WAIT_INITIAL),
                        endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                        elevator.setGoal(() -> Elevator.Goal.STOW),
                        Commands.waitUntil(() ->
                                isAtPoseWithTolerance(
                                        initialPoseSupplier.get(),
                                        initialAlignToleranceXMeters,
                                        initialAlignToleranceYMeters,
                                        initialAlignToleranceRad
                                )
                                        && Math.abs(drive.getMeasuredChassisAngularVelocityRadPerSec()) < initialAlignToleranceRadPerSecond
                        )
                )
        );

        DoubleSupplier elevatorPercentageSupplier = () -> operatorDashboard.disableInterpolateAutoAlign.get()
                ? 1
                : elevator.getPositionMeters() / elevatorGoalSupplier.get().setpointMeters.getAsDouble();
        Supplier<Pose2d> finalPoseSupplier = () -> getFinalAlignPose(elevatorPercentageSupplier.getAsDouble(), reefSideSupplier.get(), sideSupplier.get());
        Supplier<Command> driveFinal = () -> drive.moveTo(finalPoseSupplier);
        Command waitFinalAndElevator = Commands.sequence(
                Commands.parallel(
                        setGoal(Goal.AUTO_SCORE_CORAL_WAIT_FINAL),
                        endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                        elevator.setGoal(elevatorGoalSupplier),
                        Commands.waitUntil(() ->
                                isAtPoseWithTolerance(
                                        finalPoseSupplier.get(),
                                        finalAlignToleranceMeters,
                                        finalAlignToleranceRad
                                )
                                        && Math.abs(drive.getMeasuredChassisLinearVelocityMetersPerSec()) < finalAlignToleranceMetersPerSecond
                                        && Math.abs(drive.getMeasuredChassisAngularVelocityRadPerSec()) < finalAlignToleranceRadPerSecond
                        )
                ),
                Commands.parallel(
                        setGoal(Goal.AUTO_SCORE_CORAL_WAIT_ELEVATOR),
                        elevator.waitUntilAtGoal()
                ),
                Commands.waitSeconds(0.3)
        );
        // Don't allow forcing for a bit, then check if force is true
        Command waitForForce = Commands.sequence(
                Commands.parallel(
                        Commands.waitSeconds(2),
                        Commands.runOnce(() -> autoScoreForceable = false)
                ),
                Commands.parallel(
                        Commands.waitUntil(forceCondition),
                        Commands.runOnce(() -> autoScoreForceable = true)
                )
        );

        Command score = Commands.parallel(
                setGoal(Goal.AUTO_SCORE_CORAL_SCORING),
                endEffector.setGoal(EndEffector.RollersGoal.SCORE_CORAL),
                waitUntilEndEffectorNotTriggered(Commands.waitSeconds(0.5))
        );
        // Wait for coral to settle and send the elevator back down
        Command finalize = Commands.parallel(
                elevator.setGoal(() -> Elevator.Goal.STOW),
                Commands.waitSeconds(scoreCoralSettleSeconds)
        );
        if (duringAuto) {
            return Commands.sequence(
                    initial,
                    Commands.race(
                            driveFinal.get(),
                            Commands.sequence(
                                    Commands.race(
                                            waitFinalAndElevator,
                                            waitForForce
                                    ),
                                    score,
                                    finalize
                            )
                    )
            );
        } else {
            return CommandsExt.onlyIf(
                    // Only run if you have coral and are in front of your reef side
                    () -> (endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get())
                            && alignable(reefSideSupplier.get(), RobotState.get().getPose()),
                    CommandsExt.cancelOnTrigger(
                            cancelCondition,
                            Commands.sequence(
                                    initial,
                                    Commands.race(
                                            driveFinal.get(),
                                            waitFinalAndElevator,
                                            waitForForce
                                    ),
                                    // don't allow cancelling
                                    CommandsExt.schedule(Commands.race(
                                            driveFinal.get(),
                                            score.andThen(finalize)
                                    ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming))
                            )
                    )
            );
        }
    }
}