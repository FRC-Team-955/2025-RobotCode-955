package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.OperatorDashboard;
import frc.robot.OperatorDashboard.CoralScoringLevel;
import frc.robot.RobotMechanism;
import frc.robot.RobotState;
import frc.robot.Util;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;
import frc.robot.subsystems.superstructure.ReefAlign.LocalReefSide;
import frc.robot.subsystems.superstructure.ReefAlign.ReefZoneSide;
import frc.robot.subsystems.superstructure.StationAlign.Station;
import frc.robot.util.BackgroundCommandScheduler;
import frc.robot.util.commands.CommandsExt;
import frc.robot.util.subsystem.SubsystemBaseExt;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;
import static frc.robot.subsystems.superstructure.SuperstructureTuning.funnelIntakeHomeMeters;
import static frc.robot.subsystems.superstructure.SuperstructureTuning.funnelIntakeInitialMeters;

public class Superstructure extends SubsystemBaseExt {
    private final RobotState robotState = RobotState.get();
    private final RobotMechanism robotMechanism = RobotMechanism.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final Drive drive = Drive.get();
    private final Elevator elevator = Elevator.get();
    private final EndEffector endEffector = EndEffector.get();
    private final Funnel funnel = Funnel.get();
    private final GamePieceVision gamePieceVision = GamePieceVision.get();

    private final SuperstructureIO io = createIO();
    private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE,

        MANUAL_SCORE_CORAL_WAIT_ELEVATOR,
        MANUAL_SCORE_CORAL_WAIT_CONFIRM,
        MANUAL_SCORE_CORAL_SCORING,

        AUTO_SCORE_CORAL_WAIT_RAISE,
        AUTO_SCORE_CORAL_WAIT_ALIGN,
        AUTO_SCORE_CORAL_WAIT_ELEVATOR,
        AUTO_SCORE_CORAL_SCORING,

        DESCORE_ALGAE_WAIT_ELEVATOR,
        DESCORE_ALGAE_DESCORING,

        AUTO_DESCORE_ALGAE_WAIT_RAISE,
        AUTO_DESCORE_ALGAE_WAIT_ALIGN,
        AUTO_DESCORE_ALGAE_WAIT_AMPERAGE,
        AUTO_DESCORE_ALGAE_MOVE_BACK,

        HANDOFF,
        HOME,

        FUNNEL_INTAKE_WAITING,

        AUTO_FUNNEL_INTAKE_WAITING_ALIGN,
        AUTO_FUNNEL_INTAKE_WAITING_SHAKE,

        EJECT,
    }

    @Getter
    private Goal goal = Goal.IDLE;

    private Command setGoal(Goal goal) {
        return runOnce(() -> this.goal = goal);
    }

    private final BackgroundCommandScheduler backgroundCommandScheduler = new BackgroundCommandScheduler();

    private final Debouncer endEffectorBeamBreakDebouncerShort = new Debouncer(3 * 0.02);
    private final Debouncer endEffectorBeamBreakDebouncerLong = new Debouncer(0.25);

    private final Debouncer funnelBeamBreakDebouncerShort = new Debouncer(3 * 0.02);
    private final Debouncer funnelBeamBreakDebouncerLong = new Debouncer(0.25);

    private static Superstructure instance;

    public static Superstructure get() {
        if (instance == null)
            synchronized (Superstructure.class) {
                instance = new Superstructure();
            }

        return instance;
    }

    private Superstructure() {
        super(5);
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superstructure", inputs);
    }


    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Goal", goal);

        Color color = DriverStation.isDisabled()
                ? DashboardColors.disabled.get()
                : switch (goal) {
            case AUTO_SCORE_CORAL_WAIT_RAISE, AUTO_SCORE_CORAL_SCORING,
                 AUTO_FUNNEL_INTAKE_WAITING_ALIGN, AUTO_FUNNEL_INTAKE_WAITING_SHAKE,
                 AUTO_DESCORE_ALGAE_WAIT_RAISE, AUTO_DESCORE_ALGAE_MOVE_BACK -> DashboardColors.autoScoring.get();

            case AUTO_SCORE_CORAL_WAIT_ALIGN, AUTO_SCORE_CORAL_WAIT_ELEVATOR,
                 AUTO_DESCORE_ALGAE_WAIT_ALIGN, AUTO_DESCORE_ALGAE_WAIT_AMPERAGE ->
                    autoForceable ? DashboardColors.driverConfirm.get() : DashboardColors.autoScoring.get();

            case DESCORE_ALGAE_WAIT_ELEVATOR, MANUAL_SCORE_CORAL_WAIT_ELEVATOR -> DashboardColors.waitElevator.get();

            case FUNNEL_INTAKE_WAITING -> DashboardColors.funnelIntaking.get();

            case MANUAL_SCORE_CORAL_WAIT_CONFIRM -> DashboardColors.driverConfirm.get();

            case HOME, HANDOFF,
                 MANUAL_SCORE_CORAL_SCORING, DESCORE_ALGAE_DESCORING -> DashboardColors.finalizing.get();

            case EJECT -> DashboardColors.eject.get();

            case IDLE -> Color.kBlack;
        };
        Logger.recordOutput("Superstructure/Color", color.toHexString());
        robotMechanism.superstructure.color.setColor(new Color8Bit(color));

        Pose3d robotPose = new Pose3d(robotState.getPose());

        if (gamePieceVision.visibleNotDebounced()) {
            Pose3d coral = robotPose.transformBy(coralAboveFunnel);
            Logger.recordOutput("Superstructure/CoralAboveFunnel", new Pose3d[]{coral});
        } else {
            Logger.recordOutput("Superstructure/CoralAboveFunnel", new Pose3d[]{});
        }

        if (inputs.funnelBeamBreakTriggered) {
            Pose3d coral = robotPose.transformBy(coralInFunnel);
            Logger.recordOutput("Superstructure/CoralInFunnel", new Pose3d[]{coral});
        } else {
            Logger.recordOutput("Superstructure/CoralInFunnel", new Pose3d[]{});
        }

        if (inputs.endEffectorBeamBreakTriggered) {
            Pose3d coral = robotPose.transformBy(coralInEndEffector(elevator.getPositionMeters(), endEffector.getAngleRad()));
            Logger.recordOutput("Superstructure/CoralInEndEffector", new Pose3d[]{coral});
        } else {
            Logger.recordOutput("Superstructure/CoralInEndEffector", new Pose3d[]{});
        }

        Logger.recordOutput("Superstructure/ReefAlign/FinalAlign", ReefAlign.getFinalAlignPose(operatorDashboard.getSelectedReefZoneSide(), operatorDashboard.getSelectedLocalReefSide()));
    }

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

    /** Reacts quickly to change so better for waiting for the beam break */
    @AutoLogOutput(key = "Superstructure/FunnelTriggeredShort")
    private boolean funnelTriggeredShort() {
        return funnelBeamBreakDebouncerShort.calculate(inputs.funnelBeamBreakTriggered);
    }

    /** Reacts slowly to change so better for gating commands */
    @AutoLogOutput(key = "Superstructure/FunnelTriggeredLong")
    private boolean funnelTriggeredLong() {
        return funnelBeamBreakDebouncerLong.calculate(inputs.funnelBeamBreakTriggered);
    }

    private Command waitUntilEndEffectorTriggered(Command ifIgnored) {
        return Commands.either(
                ifIgnored,
                Commands.waitUntil(this::endEffectorTriggeredShort),
                operatorDashboard.ignoreEndEffectorBeamBreak::get
        );
    }

    private Command waitUntilFunnelTriggered() {
        return Commands.waitUntil(this::funnelTriggeredShort);
    }

    private Command waitUntilEndEffectorNotTriggered(Command ifIngored) {
        return Commands.either(
                ifIngored,
                Commands.waitUntil(() -> !endEffectorTriggeredShort()),
                operatorDashboard.ignoreEndEffectorBeamBreak::get
        );
    }

    public Command cancel() {
        return Commands.parallel(
                backgroundCommandScheduler.cancelIfRunning(),
                setGoal(Goal.IDLE),
                elevator.setGoal(() -> Elevator.Goal.STOW),
                endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                funnel.setGoal(Funnel.Goal.IDLE)
        );
    }

    public Command ensureNotBusyAndResetGoals() {
        return CommandsExt.eagerSequence(
                backgroundCommandScheduler.waitUntilFinish(),
                cancel()
        );
    }

    private Command wrapExposedCommand(Command command) {
        return CommandsExt.eagerSequence(
                ensureNotBusyAndResetGoals(),
                command
        );
    }

    private Command wrapExposedCommand(Command whileWaiting, Command command) {
        return CommandsExt.eagerSequence(
                ensureNotBusyAndResetGoals().deadlineFor(whileWaiting),
                command
        );
    }

    private Command funnelSetGoalIntakeAlternate() {
        Timer funnelTimer = new Timer();
        return funnel.startRun(
                funnelTimer::restart,
                () -> {
                    boolean backwards = funnelTimer.hasElapsed(0.92);
                    if (backwards) {
                        funnel.setGoalInstantaneous(Funnel.Goal.INTAKE_BACKWARDS);
                        funnelTimer.advanceIfElapsed(1.0);
                    } else {
                        funnel.setGoalInstantaneous(Funnel.Goal.INTAKE_FORWARDS);
                    }
                }
        );
    }

    private Command handoffAndHome() {
        return CommandsExt.eagerSequence(
                waitUntilEndEffectorTriggered(Commands.none())
                        .deadlineFor(Commands.parallel(
                                setGoal(Goal.HANDOFF),
                                endEffector.setGoal(EndEffector.RollersGoal.FUNNEL_INTAKE),
                                funnelSetGoalIntakeAlternate()
                        )),
                Commands.parallel(
                        setGoal(Goal.HOME),
                        CommandsExt.eagerSequence(
                                endEffector.moveByAndWaitUntilDone(funnelIntakeInitialMeters::get),
                                endEffector.setGoal(EndEffector.RollersGoal.ZERO_CORAL),
                                Commands.waitSeconds(0.1),
                                endEffector.moveByAndWaitUntilDone(funnelIntakeHomeMeters::get)
                        ).deadlineFor(elevator.zeroCoral()),
                        funnel.setGoal(Funnel.Goal.IDLE)
                )
        );
    }

    private Command shake() {
        return drive.runRobotRelative(() -> Timer.getTimestamp() % 0.25 < 0.125
                ? new ChassisSpeeds(-0.05, -0.05, -0.3)
                : new ChassisSpeeds(0.05, 0.05, 0.3));
    }

    public Command eject() {
        Timer funnelTimer = new Timer();
        return wrapExposedCommand(Commands.parallel(
                setGoal(Goal.EJECT),
                endEffector.setGoal(EndEffector.RollersGoal.EJECT),
                funnel.startRun(
                        funnelTimer::restart,
                        () -> {
                            boolean backwards = funnelTimer.hasElapsed(0.86);
                            if (backwards) {
                                funnel.setGoalInstantaneous(Funnel.Goal.EJECT_BACKWARDS);
                                funnelTimer.advanceIfElapsed(1.0);
                            } else {
                                funnel.setGoalInstantaneous(Funnel.Goal.EJECT_FORWARDS);
                            }
                        }
                )
        ));
    }

    public Command scoreCoralManual(
            boolean duringAuto,
            BooleanSupplier forwardCondition,
            Supplier<CoralScoringLevel> coralScoringLevelSupplier
    ) {
        Command raiseElevator = Commands.parallel(
                setGoal(Goal.MANUAL_SCORE_CORAL_WAIT_ELEVATOR),
                endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                elevator.setGoalAndWaitUntilAtGoal(() -> coralScoringLevelSupplier.get().coralScoringElevatorGoal)
        );

        Command waitConfirm = Commands.parallel(
                setGoal(Goal.MANUAL_SCORE_CORAL_WAIT_CONFIRM),
                Commands.waitUntil(forwardCondition)
        );

        Command driveWhileScoringL1 = CommandsExt.onlyIf(
                () -> coralScoringLevelSupplier.get() == CoralScoringLevel.L1,
                drive.runRobotRelative(() -> new ChassisSpeeds(0, -1.0, 0)).asProxy()
        );

        Command score = Commands.parallel(
                setGoal(Goal.MANUAL_SCORE_CORAL_SCORING),
                Commands.either(
                        endEffector.setGoal(EndEffector.RollersGoal.SCORE_CORAL_L1),
                        endEffector.setGoal(EndEffector.RollersGoal.SCORE_CORAL),
                        () -> coralScoringLevelSupplier.get() == CoralScoringLevel.L1
                ),
                elevator.setGoal(() -> coralScoringLevelSupplier.get().coralScoringElevatorGoal),
                waitUntilEndEffectorNotTriggered(Commands.waitSeconds(0.5))
        );

        // Wait for coral to settle
        Command finalize = Commands.either(
                Commands.waitSeconds(scoreCoralL1SettleSeconds),
                Commands.waitSeconds(scoreCoralSettleSeconds),
                () -> coralScoringLevelSupplier.get() == CoralScoringLevel.L1
        );

        if (duringAuto) {
            return wrapExposedCommand(CommandsExt.eagerSequence(
                    raiseElevator,
                    waitConfirm,
                    CommandsExt.eagerSequence(
                            score,
                            finalize
                    ).deadlineFor(driveWhileScoringL1)
            ));
        } else {
            return wrapExposedCommand(CommandsExt.onlyIf(
                    () -> endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                    CommandsExt.eagerSequence(
                            raiseElevator,
                            waitConfirm,
                            backgroundCommandScheduler.scheduleInBackground(CommandsExt.eagerSequence(
                                    score,
                                    finalize
                            ).deadlineFor(driveWhileScoringL1))
                    )
            ));
        }
    }

    public Command descoreAlgaeManual(Supplier<ReefZoneSide> reefZoneSideSupplier) {
        return wrapExposedCommand(CommandsExt.onlyIf(
                () -> !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                CommandsExt.eagerSequence(
                        Commands.parallel(
                                setGoal(Goal.DESCORE_ALGAE_WAIT_ELEVATOR),
                                endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                                elevator.setGoalAndWaitUntilAtGoal(() -> reefZoneSideSupplier.get().algaeDescoringElevatorGoal)
                        ),
                        Commands.parallel(
                                setGoal(Goal.DESCORE_ALGAE_DESCORING),
                                endEffector.setGoal(EndEffector.RollersGoal.DESCORE_ALGAE),
                                elevator.setGoal(() -> reefZoneSideSupplier.get().algaeDescoringElevatorGoal),
                                Commands.idle()
                        )
                )
        ));
    }

    public Command funnelIntake(boolean duringAuto) {
        Command intake = Commands.race(
                waitUntilEndEffectorTriggered(Commands.idle()),
                waitUntilFunnelTriggered()
        ).deadlineFor(
                setGoal(Goal.FUNNEL_INTAKE_WAITING),
                endEffector.setGoal(EndEffector.RollersGoal.FUNNEL_INTAKE),
                funnelSetGoalIntakeAlternate()
        );
        if (duringAuto) {
            return wrapExposedCommand(CommandsExt.onlyIf(
                    () -> !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                    CommandsExt.eagerSequence(
                            intake,
                            backgroundCommandScheduler.scheduleInBackground(handoffAndHome())
                    )
            ));
        } else {
            return wrapExposedCommand(CommandsExt.onlyIf(
                    () -> !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                    CommandsExt.eagerSequence(
                            intake,
                            backgroundCommandScheduler.scheduleInBackground(handoffAndHome())
                    )
            ));
        }
    }

    public Command autoFunnelIntake(boolean duringAuto, Station station) {
        Command intake = Commands.race(
                waitUntilEndEffectorTriggered(Commands.idle()),
                waitUntilFunnelTriggered(),
                gamePieceVision.waitForGamePiece()
        ).deadlineFor(
                endEffector.setGoal(EndEffector.RollersGoal.FUNNEL_INTAKE),
                funnelSetGoalIntakeAlternate(),
                CommandsExt.eagerSequence(
                        Commands.parallel(
                                setGoal(Goal.AUTO_FUNNEL_INTAKE_WAITING_ALIGN),
                                drive.moveTo(station::getAlignPose)
                                        .until(() -> StationAlign.atAlignPose(robotState.getPose(), station))
                        ),
                        Commands.parallel(
                                setGoal(Goal.AUTO_FUNNEL_INTAKE_WAITING_SHAKE),
                                shake()
                        )
                )
        );
        if (duringAuto) {
            return wrapExposedCommand(
                    drive.stop(),
                    CommandsExt.onlyIf(
                            () -> !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                            CommandsExt.eagerSequence(
                                    intake,
                                    backgroundCommandScheduler.scheduleInBackground(handoffAndHome())
                            )
                    )
            );
        } else {
            return wrapExposedCommand(
                    drive.stop(),
                    CommandsExt.onlyIf(
                            () -> !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                            CommandsExt.eagerSequence(
                                    intake,
                                    backgroundCommandScheduler.scheduleInBackground(handoffAndHome())
                            )
                    )
            );
        }
    }

    @Getter
    private boolean autoForceable = false;

    public Command autoScoreCoral(
            boolean duringAuto,
            Supplier<ReefZoneSide> reefSideSupplier,
            Supplier<LocalReefSide> sideSupplier,
            Supplier<CoralScoringLevel> coralScoringLevelSupplier,
            BooleanSupplier forceCondition
    ) {
        DoubleSupplier elevatorPercentageSupplier = () -> operatorDashboard.disableInterpolateAutoAlign.get()
                ? 1
                : elevator.getPositionMeters() / coralScoringLevelSupplier.get().coralScoringElevatorGoal.setpointMeters.getAsDouble();
        Supplier<Pose2d> alignPoseSupplier = () -> ReefAlign.getAlignPose(robotState.getPose(), elevatorPercentageSupplier.getAsDouble(), reefSideSupplier.get(), sideSupplier.get());

        Command initial = Commands.race(
                // Drive to initial position
                drive.moveTo(alignPoseSupplier),
                Commands.parallel(
                        setGoal(Goal.AUTO_SCORE_CORAL_WAIT_RAISE),
                        endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                        elevator.setGoal(() -> Elevator.Goal.STOW),
                        Commands.waitUntil(() -> ReefAlign.canRaiseElevator(robotState.getPose(), reefSideSupplier.get(), sideSupplier.get()))
                )
        );

        Command waitFinalAndElevator = CommandsExt.eagerSequence(
                Commands.parallel(
                        setGoal(Goal.AUTO_SCORE_CORAL_WAIT_ALIGN),
                        endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                        elevator.setGoal(() -> coralScoringLevelSupplier.get().coralScoringElevatorGoal),
                        Commands.waitUntil(() -> ReefAlign.atFinalAlign(robotState.getPose(), drive.getMeasuredChassisSpeeds(), reefSideSupplier.get(), sideSupplier.get()))
                ),
                Commands.parallel(
                        setGoal(Goal.AUTO_SCORE_CORAL_WAIT_ELEVATOR),
                        elevator.waitUntilAtGoal()
                )
//                Commands.waitSeconds(0.3)
        );
        // Don't allow forcing for a bit, then check if force is true
        Command waitForForce = CommandsExt.eagerSequence(
                Commands.parallel(
                        Commands.waitSeconds(2),
                        Commands.runOnce(() -> autoForceable = false)
                ),
                Commands.parallel(
                        Commands.waitUntil(forceCondition),
                        Commands.runOnce(() -> autoForceable = true)
                ).deadlineFor(
                        // We only want to offset the elevator position if we aren't aligned and are taking a while to align
                        elevator.setDistanceFromScoringPositionContinuous(
                                () -> robotState.getPose().getTranslation()
                                        .getDistance(ReefAlign.getFinalAlignPose(reefSideSupplier.get(), sideSupplier.get()).getTranslation())
                        )
                )
        );

        Command score = Commands.parallel(
                setGoal(Goal.AUTO_SCORE_CORAL_SCORING),
                Commands.either(
                        endEffector.setGoal(EndEffector.RollersGoal.SCORE_CORAL_L1),
                        endEffector.setGoal(EndEffector.RollersGoal.SCORE_CORAL),
                        () -> coralScoringLevelSupplier.get() == CoralScoringLevel.L1
                ),
                waitUntilEndEffectorNotTriggered(Commands.waitSeconds(0.5))
        );
        // Wait for coral to settle and send the elevator back down
        Command finalize = Commands.parallel(
                elevator.setGoal(() -> Elevator.Goal.STOW),
                Commands.either(
                        Commands.waitSeconds(scoreCoralL1SettleSeconds),
                        Commands.waitSeconds(scoreCoralSettleSeconds),
                        () -> coralScoringLevelSupplier.get() == CoralScoringLevel.L1
                )
        );
        if (duringAuto) {
            return wrapExposedCommand(
                    CommandsExt.eagerSequence(
                            drive.moveTo(alignPoseSupplier)
                                    // We don't really care about position tolerances right now,
                                    // checking velocity is a good way to approximate "we're at the position we want"
                                    .until(() -> Util.isWithinVelocityTolerance(drive.getMeasuredChassisSpeeds(), 0.2, Units.degreesToRadians(15))),
                            shake()
                    ),
                    CommandsExt.eagerSequence(
                            initial,
                            Commands.race(
                                    drive.moveTo(alignPoseSupplier),
                                    CommandsExt.eagerSequence(
                                            Commands.race(
                                                    waitFinalAndElevator,
                                                    waitForForce
                                            ),
                                            score,
                                            finalize
                                    )
                            )
                    )
            );
        } else
            return wrapExposedCommand(
                    drive.stop(),
                    CommandsExt.onlyIf(
                            // Only run if you have coral and are in front of your reef side
                            () -> (endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get())
                                    && ReefAlign.isAlignable(robotState.getPose(), reefSideSupplier.get()),
                            CommandsExt.eagerSequence(
                                    initial,
                                    Commands.race(
                                            drive.moveTo(alignPoseSupplier),
                                            waitFinalAndElevator,
                                            waitForForce
                                    ),
                                    backgroundCommandScheduler.scheduleInBackground(Commands.race(
                                            drive.moveTo(alignPoseSupplier),
                                            CommandsExt.eagerSequence(
                                                    score,
                                                    finalize
                                            )
                                    ))
                            )
                    )
            );
    }

    public Command autoDescoreAlgae(
            Supplier<ReefZoneSide> reefSideSupplier,
            BooleanSupplier forceCondition
    ) {
        Command driveTo = Commands.race(
                // Drive to position
                drive.moveTo(() -> ReefAlign.getDescoreAlignPose(reefSideSupplier.get())),
                CommandsExt.eagerSequence(
                        Commands.parallel(
                                setGoal(Goal.AUTO_DESCORE_ALGAE_WAIT_RAISE),
                                endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                                elevator.setGoal(() -> Elevator.Goal.STOW),
                                Commands.waitUntil(() -> ReefAlign.descoreCanRaiseElevator(robotState.getPose(), reefSideSupplier.get()))
                        ),
                        Commands.parallel(
                                setGoal(Goal.AUTO_DESCORE_ALGAE_WAIT_ALIGN),
                                endEffector.setGoal(EndEffector.RollersGoal.DESCORE_ALGAE),
                                elevator.setGoal(() -> reefSideSupplier.get().algaeDescoringElevatorGoal),
                                Commands.waitUntil(() -> ReefAlign.descoreIsAligned(robotState.getPose(), reefSideSupplier.get()))
                        )
                )
        );

        Command waitAlgae = Commands.parallel(
                Commands.race(
                        drive.runRobotRelative(() -> new ChassisSpeeds(-0.4, 0, 0)),
                        CommandsExt.eagerSequence(
                                Commands.waitSeconds(0.5),
                                endEffector.waitUntilDescoreAlgaeAmperageTriggered()
                        )
                ),
                setGoal(Goal.AUTO_DESCORE_ALGAE_WAIT_AMPERAGE)
        );

        Timer driveBackTimer = new Timer();
        Command driveBack = drive.runRobotRelative(() -> new ChassisSpeeds(driveBackTimer.get() * 4.0, 0, 0))
                .withTimeout(0.5)
                .deadlineFor(
                        Commands.runOnce(driveBackTimer::restart),
                        setGoal(Goal.AUTO_DESCORE_ALGAE_MOVE_BACK)
                );

        Command waitForForce = CommandsExt.eagerSequence(
                Commands.parallel(
                        Commands.waitSeconds(2),
                        Commands.runOnce(() -> autoForceable = false)
                ),
                Commands.parallel(
                        Commands.waitUntil(forceCondition),
                        Commands.runOnce(() -> autoForceable = true)
                )
        );

        return wrapExposedCommand(
                drive.stop(),
                CommandsExt.onlyIf(
                        () -> (!endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get())
                                && ReefAlign.isAlignable(robotState.getPose(), reefSideSupplier.get()),
                        CommandsExt.eagerSequence(
                                Commands.race(
                                        CommandsExt.eagerSequence(
                                                driveTo,
                                                waitAlgae
                                        ),
                                        waitForForce
                                ),
                                driveBack
                        )
                )
        );
    }
}