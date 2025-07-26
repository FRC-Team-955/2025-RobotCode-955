package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.lib.subsystem.CommandBasedSubsystem;
import frc.robot.OperatorDashboard;
import frc.robot.OperatorDashboard.CoralScoringLevel;
import frc.robot.RobotMechanism;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;
import frc.robot.subsystems.superstructure.ReefAlign.LocalReefSide;
import frc.robot.subsystems.superstructure.ReefAlign.ReefZoneSide;
import frc.robot.subsystems.superstructure.StationAlign.Station;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;

public class Superstructure extends CommandBasedSubsystem {
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

    private record Goals(
            Elevator.Goal elevatorGoal,
            EndEffector.Goal endEffectorGoal,
            Funnel.Goal funnelGoal
    ) {}

    @RequiredArgsConstructor
    public enum Goal {
        IDLE(s -> new Goals(Elevator.Goal.STOW, EndEffector.Goal.IDLE, Funnel.Goal.IDLE)),

        MANUAL_SCORE_CORAL_WAIT_FOR_ELEVATOR(s -> new Goals(s.level.get().coralScoringElevatorGoal, EndEffector.Goal.IDLE, Funnel.Goal.IDLE)),
        MANUAL_SCORE_CORAL_WAIT_FOR_CONFIRM(MANUAL_SCORE_CORAL_WAIT_FOR_ELEVATOR.goals),
        MANUAL_SCORE_CORAL_SCORING(s -> new Goals(
                s.level.get().coralScoringElevatorGoal,
                s.level.get() == CoralScoringLevel.L1 ? EndEffector.Goal.SCORE_CORAL_L1 : EndEffector.Goal.SCORE_CORAL,
                Funnel.Goal.IDLE
        )),

        AUTO_SCORE_CORAL_WAIT_UNTIL_CAN_RAISE(IDLE.goals),
        AUTO_SCORE_CORAL_WAIT_FOR_ALIGN(MANUAL_SCORE_CORAL_WAIT_FOR_ELEVATOR.goals),
        AUTO_SCORE_CORAL_WAIT_FOR_ELEVATOR(AUTO_SCORE_CORAL_WAIT_FOR_ALIGN.goals),
        AUTO_SCORE_CORAL_WAIT_BEFORE_SCORING(AUTO_SCORE_CORAL_WAIT_FOR_ELEVATOR.goals),
        AUTO_SCORE_CORAL_SCORING(MANUAL_SCORE_CORAL_SCORING.goals),

        DESCORE_ALGAE_WAIT_FOR_ELEVATOR(s -> new Goals(s.side.get().algaeDescoringElevatorGoal, EndEffector.Goal.IDLE, Funnel.Goal.IDLE)),
        DESCORE_ALGAE_DESCORING(s -> new Goals(s.side.get().algaeDescoringElevatorGoal, EndEffector.Goal.DESCORE_ALGAE, Funnel.Goal.IDLE)),

        AUTO_DESCORE_ALGAE_WAIT_UNTIL_CAN_RAISE(IDLE.goals),
        AUTO_DESCORE_ALGAE_WAIT_FOR_ALIGN(DESCORE_ALGAE_DESCORING.goals),
        AUTO_DESCORE_ALGAE_WAIT_FOR_AMPERAGE(AUTO_DESCORE_ALGAE_WAIT_FOR_ALIGN.goals),
        AUTO_DESCORE_ALGAE_MOVE_BACK(AUTO_DESCORE_ALGAE_WAIT_FOR_AMPERAGE.goals),

        HANDOFF(s ->
                s.operatorDashboard.ignoreEndEffectorBeamBreak.get()
                        ? new Goals(Elevator.Goal.STOW, EndEffector.Goal.FUNNEL_INTAKE_MANUAL, Funnel.Goal.INTAKE_ALTERNATE)
                        : new Goals(Elevator.Goal.STOW, EndEffector.Goal.FUNNEL_INTAKE, Funnel.Goal.INTAKE_ALTERNATE)
        ),
        HOME_STEP_1(s -> new Goals(Elevator.Goal.ZERO_CORAL, EndEffector.Goal.HOME_INITIAL, Funnel.Goal.IDLE)),
        HOME_STEP_2(s -> new Goals(Elevator.Goal.ZERO_CORAL, EndEffector.Goal.ZERO_CORAL, Funnel.Goal.IDLE)),
        HOME_STEP_3(s -> new Goals(Elevator.Goal.STOW, EndEffector.Goal.IDLE, Funnel.Goal.IDLE)),
        HOME_STEP_4(s -> new Goals(Elevator.Goal.STOW, EndEffector.Goal.fHOME_FINAL, Funnel.Goal.IDLE)),

        FUNNEL_INTAKE_WAITING(HANDOFF.goals),

        AUTO_FUNNEL_INTAKE_WAITING_ALIGN(FUNNEL_INTAKE_WAITING.goals),
        AUTO_FUNNEL_INTAKE_WAITING_SHAKE(AUTO_FUNNEL_INTAKE_WAITING_ALIGN.goals),

        EJECT(s -> new Goals(Elevator.Goal.STOW, EndEffector.Goal.EJECT_ALTERNATE, Funnel.Goal.EJECT_ALTERNATE)),

        ZERO_ELEVATOR(s -> {throw new RuntimeException("TODO SEE ELEVATOR JOYSTICK CONTROL");}),
        ;

        public final Function<Superstructure, Goals> goals;
    }

    @Getter
    private Goal goal = Goal.IDLE;

    private Command setGoal(Goal goal) {
        return runOnce(() -> this.goal = goal);
    }

    // Goal context
    // TODO
    private Supplier<CoralScoringLevel> level;
    private Supplier<ReefZoneSide> side;

    private final Debouncer endEffectorBeamBreakDebouncerShort = new Debouncer(3 * 0.02);
    private final Debouncer endEffectorBeamBreakDebouncerLong = new Debouncer(0.25);

    private final Debouncer funnelBeamBreakDebouncerShort = new Debouncer(3 * 0.02);
    private final Debouncer funnelBeamBreakDebouncerLong = new Debouncer(0.25);

    private final Debouncer hasCoralDebouncer = new Debouncer(1, Debouncer.DebounceType.kFalling);

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

        // OperatorDashboard periodicBeforeCommands runs after superstructure
        throw new RuntimeException("TODO update");
//        operatorDashboard.setIgnoreClosestReefSideChanges(switch (goal) {
//            case AUTO_SCORE_CORAL_WAIT_ALIGN, AUTO_SCORE_CORAL_WAIT_ELEVATOR, AUTO_SCORE_CORAL_SCORING,
//                 AUTO_DESCORE_ALGAE_WAIT_FOR_ALIGN, AUTO_DESCORE_ALGAE_WAIT_FOR_AMPERAGE,
//                 AUTO_DESCORE_ALGAE_MOVE_BACK -> true;
//
//            // Allow reef side changes before elevator raises during auto align sequences
//            case AUTO_SCORE_CORAL_WAIT_RAISE, AUTO_DESCORE_ALGAE_WAIT_UNTIL_CAN_RAISE,
//                 // All goals that don't involve auto choose side
//                 IDLE,
//                 MANUAL_SCORE_CORAL_WAIT_FOR_ELEVATOR, MANUAL_SCORE_CORAL_WAIT_FOR_CONFIRM, MANUAL_SCORE_CORAL_SCORING,
//                 DESCORE_ALGAE_WAIT_FOR_ELEVATOR, DESCORE_ALGAE_DESCORING,
//                 HANDOFF, HOME,
//                 FUNNEL_INTAKE_WAITING,
//                 AUTO_FUNNEL_INTAKE_WAITING_ALIGN, AUTO_FUNNEL_INTAKE_WAITING_SHAKE,
//                 EJECT,
//                 ZERO_ELEVATOR -> false;
//        });
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Goal", goal);
        Goals goals = goal.goals.apply(this);
        elevator.setGoal(goals.elevatorGoal);
        endEffector.setGoal(goals.endEffectorGoal);
        funnel.setGoal(goals.funnelGoal);

        Logger.recordOutput("Superstructure/Forceable", forceable);
        Logger.recordOutput("Superstructure/WasForced", wasForced);

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

    @AutoLogOutput(key = "Superstructure/HasCoral")
    private boolean hasCoral() {
        return hasCoralDebouncer.calculate(endEffectorTriggeredShort() || funnelTriggeredShort() || gamePieceVision.visibleDebounced());
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

    private Command waitUntilHasNoCoral() {
        return Commands.waitUntil(() -> !hasCoral());
    }

    public Command cancel() {
        return setGoal(Goal.IDLE);
    }

    private Command handoff() {
        return waitUntilEndEffectorTriggered(Commands.none())
                .deadlineFor(setGoal(Goal.HANDOFF));
    }

    public Command home() {
        return CommandsExt.eagerSequence(
                CommandsExt.onlyIf(
                        () -> !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                        CommandsExt.eagerSequence(
                                setGoal(Goal.HOME_STEP_1),
                                endEffector.moveByAndWaitUntilDone(() -> 0) // WAIT UNTIL AT SETPOINT
                        )
                ),

                setGoal(Goal.HOME_STEP_2),
                Commands.waitSeconds(0.15),

                setGoal(Goal.HOME_STEP_3),
                Commands.waitSeconds(0.05),

                setGoal(Goal.HOME_STEP_4),
                endEffector.moveByAndWaitUntilDone(() -> 0) // WAIT UNTIL AT SETPOINT
        );
    }

    public Command zeroElevator() {
        throw new RuntimeException("TODO joystick zeroing see goal and elevator");
//        return wrapExposedCommand(Commands.parallel(
//                setGoal(Goal.ZERO_ELEVATOR),
//                elevator.zeroElevator()
//        ));
    }

    private Command shake() {
        return drive.runRobotRelative(() -> Timer.getTimestamp() % 0.25 < 0.125
                ? new ChassisSpeeds(-0.05, -0.05, -0.3)
                : new ChassisSpeeds(0.05, 0.05, 0.3));
    }

    public Command eject() {
        return setGoal(Goal.EJECT);
    }

    public Command scoreCoralManual(
            BooleanSupplier forwardCondition,
            Supplier<CoralScoringLevel> coralScoringLevelSupplier
    ) {
        throw new RuntimeException("TODO set suppliers");
//        Command waitConfirm = Commands.parallel(
//                setGoal(Goal.MANUAL_SCORE_CORAL_WAIT_FOR_CONFIRM),
        // TODO rumble controller
//                Commands.waitUntil(forwardCondition)
//        );
//
//        Command driveWhileScoringL1 = CommandsExt.onlyIf(
//                () -> coralScoringLevelSupplier.get() == CoralScoringLevel.L1,
//                drive.runRobotRelative(() -> new ChassisSpeeds(0, -1.0, 0)).asProxy()
//        );
//
//        Command score = Commands.parallel(
//                setGoal(Goal.MANUAL_SCORE_CORAL_SCORING),
//                waitUntilEndEffectorNotTriggered(Commands.waitSeconds(0.5))
//        );
//
//        // Wait for coral to settle
//        Command finalize = Commands.either(
//                Commands.waitSeconds(scoreCoralL1SettleSeconds),
//                Commands.waitSeconds(scoreCoralSettleSeconds),
//                () -> coralScoringLevelSupplier.get() == CoralScoringLevel.L1
//        );
//
//        return CommandsExt.eagerSequence(
//                setGoal(Goal.MANUAL_SCORE_CORAL_WAIT_FOR_ELEVATOR),
//                elevator.waitUntilAtGoal(),
//                waitConfirm,
//                CommandsExt.eagerSequence(
//                        score,
//                        finalize
//                ).deadlineFor(driveWhileScoringL1)
//        );
    }

    public Command descoreAlgaeManual(Supplier<ReefZoneSide> reefZoneSideSupplier) {
        throw new RuntimeException("TODO set suppliers");
//        return CommandsExt.eagerSequence(
//                setGoal(Goal.DESCORE_ALGAE_WAIT_FOR_ELEVATOR),
//                elevator.waitUntilAtGoal(),
//                setGoal(Goal.DESCORE_ALGAE_DESCORING),
//                Commands.idle()
//        );
    }

    public Command funnelIntake() {
        Command intake = Commands.race(
                waitUntilEndEffectorTriggered(Commands.idle()),
                waitUntilFunnelTriggered()
        ).deadlineFor(setGoal(Goal.FUNNEL_INTAKE_WAITING));
        return CommandsExt.eagerSequence(
                intake,
                handoff(),
                home()
        );
    }

    public Command autoFunnelIntake(Station station) {
        Command intake = Commands.race(
                waitUntilEndEffectorTriggered(Commands.idle()),
                waitUntilFunnelTriggered(),
                gamePieceVision.waitForGamePiece()
        ).deadlineFor(
                CommandsExt.eagerSequence(
                        Commands.parallel(
                                setGoal(Goal.AUTO_FUNNEL_INTAKE_WAITING_ALIGN),
                                drive.moveTo(station::getAlignPose, () -> false)
                                        .until(() -> StationAlign.atAlignPose(robotState.getPose(), station))
                        ),
                        Commands.parallel(
                                setGoal(Goal.AUTO_FUNNEL_INTAKE_WAITING_SHAKE),
                                shake()
                        )
                )
        );
        return CommandsExt.eagerSequence(
                intake,
                handoff(),
                home()
        );
    }

    @Getter
    private boolean forceable = false;
    private boolean wasForced = false;

    public Command autoScoreCoral(
            Supplier<ReefZoneSide> reefSideSupplier,
            Supplier<LocalReefSide> sideSupplier,
            Supplier<CoralScoringLevel> coralScoringLevelSupplier,
            BooleanSupplier forceCondition,
            boolean safe
    ) {
        throw new RuntimeException("TODO set suppliers");
//        DoubleSupplier elevatorPercentageSupplier = () -> elevator.getPositionMeters() / coralScoringLevelSupplier.get().coralScoringElevatorGoal.value.getAsDouble();
//        Supplier<Pose2d> alignPoseSupplier = () -> ReefAlign.getAlignPose(robotState.getPose(), elevatorPercentageSupplier.getAsDouble(), reefSideSupplier.get(), sideSupplier.get());
//
//        Command initial = Commands.race(
//                // Drive to initial position
//                drive.moveTo(alignPoseSupplier, () -> false),
//                Commands.parallel(
//                        setGoal(Goal.AUTO_SCORE_CORAL_WAIT_UNTIL_CAN_RAISE),
//                        Commands.waitUntil(() -> ReefAlign.canRaiseElevator(robotState.getPose(), reefSideSupplier.get(), sideSupplier.get()))
//                )
//        );
//
//        Command waitFinalAndElevator = CommandsExt.eagerSequence(
//                Commands.parallel(
//                        setGoal(Goal.AUTO_SCORE_CORAL_WAIT_FOR_ALIGN),
//                        Commands.waitUntil(() -> ReefAlign.atFinalAlign(robotState.getPose(), drive.getMeasuredChassisSpeeds(), reefSideSupplier.get(), sideSupplier.get()))
//                ),
//                Commands.parallel(
//                        setGoal(Goal.AUTO_SCORE_CORAL_WAIT_FOR_ELEVATOR),
//                        elevator.waitUntilAtGoal()
//                )
//        );
//        // Don't allow forcing for a bit, then check if force is true
//        Command waitForForce = CommandsExt.eagerSequence(
//                Commands.runOnce(() -> {
//                    forceable = false;
//                    wasForced = false;
//                }),
//                Commands.waitSeconds(2),
//                Commands.parallel(
//                        Commands.waitUntil(forceCondition),
//                        Commands.runOnce(() -> forceable = true)
//                ).deadlineFor(
//                        // We only want to offset the elevator position if we aren't aligned and are taking a while to align
//                        elevator.setDistanceFromScoringPositionContinuous(
//                                () -> robotState.getPose().getTranslation()
//                                        .getDistance(ReefAlign.getFinalAlignPose(reefSideSupplier.get(), sideSupplier.get()).getTranslation())
//                        )
//                ),
//                Commands.runOnce(() -> wasForced = true)
//        );
//
//        Command score = CommandsExt.eagerSequence(
//                setGoal(Goal.AUTO_SCORE_CORAL_WAIT_BEFORE_SCORING),
//                safe
//                        ? Commands.waitSeconds(1)
//                        : Commands.waitSeconds(0.3),
//                setGoal(Goal.AUTO_SCORE_CORAL_SCORING),
//                waitUntilEndEffectorNotTriggered(Commands.waitSeconds(0.5))
//        );
//        // Wait for coral to settle and send the elevator back down
//        Command finalize = CommandsExt.eagerSequence(
//                Commands.either(
//                        Commands.waitSeconds(scoreCoralForceSettleSeconds),
//                        Commands.either(
//                                Commands.waitSeconds(scoreCoralL1SettleSeconds),
//                                Commands.waitSeconds(scoreCoralSettleSeconds),
//                                () -> coralScoringLevelSupplier.get() == CoralScoringLevel.L1
//                        ),
//                        () -> wasForced
//                )
//        );
        // TODO do this during auto while waiting
//                    CommandsExt.eagerSequence(
//                            drive.moveTo(alignPoseSupplier, () -> false)
//                                    // We don't really care about position tolerances right now,
//                                    // checking velocity is a good way to approximate "we're at the position we want"
//                                    .until(() -> Util.isWithinVelocityTolerance(drive.getMeasuredChassisSpeeds(), 0.2, Units.degreesToRadians(15))),
//                            Commands.parallel(
//                                    shake(),
//                                    CommandsExt.eagerSequence(
//                                            waitUntilHasNoCoral(),
//                                            Commands.runOnce(() -> shouldCancel.val = true)
//                                    )
//                            )
//                    ),
        // TODO figure out which version to use - first is auto, second is teleop
//                    CommandsExt.eagerSequence(
//                            initial,
//                            Commands.race(
//                                    drive.moveTo(alignPoseSupplier, () -> false),
//                                    CommandsExt.eagerSequence(
//                                            Commands.race(
//                                                    waitFinalAndElevator,
//                                                    waitForForce
//                                            ),
//                                            score,
//                                            finalize
//                                    )
//                            )
//                    );
//                            CommandsExt.eagerSequence(
//                                    initial,
//                                    Commands.race(
//                                            waitFinalAndElevator,
//                                            waitForForce,
//                                            // move to must execute after waitForForce so that we forceable gets reset to false before move to
//                                            drive.moveTo(alignPoseSupplier, () -> forceable)
//                                    ),
//                                    backgroundCommandScheduler.scheduleInBackground(Commands.race(
//                                            drive.moveTo(alignPoseSupplier, () -> false),
//                                            CommandsExt.eagerSequence(
//                                                    score,
//                                                    finalize
//                                            )
//                                    ))
//                            )
    }

    public Command autoDescoreAlgae(
            Supplier<ReefZoneSide> reefSideSupplier,
            BooleanSupplier forceCondition
    ) {
        throw new RuntimeException("TODO set suppliers");
//        Command driveTo = Commands.race(
//                // Drive to position
//                drive.moveTo(() -> ReefAlign.getDescoreAlignPose(reefSideSupplier.get()), () -> false),
//                CommandsExt.eagerSequence(
//                        Commands.parallel(
//                                setGoal(Goal.AUTO_DESCORE_ALGAE_WAIT_UNTIL_CAN_RAISE),
//                                Commands.waitUntil(() -> ReefAlign.descoreCanRaiseElevator(robotState.getPose(), reefSideSupplier.get()))
//                        ),
//                        Commands.parallel(
//                                setGoal(Goal.AUTO_DESCORE_ALGAE_WAIT_FOR_ALIGN),
//                                Commands.waitUntil(() -> ReefAlign.descoreIsAligned(robotState.getPose(), reefSideSupplier.get()))
//                        )
//                )
//        );
//
//        Command waitAlgae = Commands.parallel(
//                Commands.race(
//                        drive.runRobotRelative(() -> new ChassisSpeeds(-0.4, 0, 0)),
//                        CommandsExt.eagerSequence(
//                                Commands.waitSeconds(0.5),
//                                endEffector.waitUntilDescoreAlgaeAmperageTriggered()
//                        )
//                ),
//                setGoal(Goal.AUTO_DESCORE_ALGAE_WAIT_FOR_AMPERAGE)
//        );
//
//        Timer driveBackTimer = new Timer();
//        Command driveBack = drive.runRobotRelative(() -> new ChassisSpeeds(driveBackTimer.get() * 4.0, 0, 0))
//                .withTimeout(0.5)
//                .deadlineFor(
//                        Commands.runOnce(driveBackTimer::restart),
//                        setGoal(Goal.AUTO_DESCORE_ALGAE_MOVE_BACK)
//                );
//
//        Command waitForForce = CommandsExt.eagerSequence(
//                Commands.runOnce(() -> {
//                    forceable = false;
//                    wasForced = false;
//                }),
//                Commands.waitSeconds(2),
//                Commands.parallel(
//                        Commands.waitUntil(forceCondition),
//                        Commands.runOnce(() -> forceable = true)
//                ),
//                Commands.runOnce(() -> wasForced = true)
//        );
//
//        return CommandsExt.eagerSequence(
//                Commands.race(
//                        CommandsExt.eagerSequence(
//                                driveTo,
//                                waitAlgae
//                        ),
//                        waitForForce
//                ),
//                driveBack
//        );
    }
}