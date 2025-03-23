package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.OperatorDashboard;
import frc.robot.OperatorDashboard.LocalReefSide;
import frc.robot.OperatorDashboard.ReefZoneSide;
import frc.robot.RobotMechanism;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.funnel.Funnel;
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

import static frc.robot.subsystems.superstructure.AutoAlignLocations.*;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;
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

    @RequiredArgsConstructor
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

        AUTO_DESCORE_ALGAE_WAIT_INITIAL,
        AUTO_DESCORE_ALGAE_WAIT_FINAL,
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
            case AUTO_SCORE_CORAL_WAIT_INITIAL, AUTO_SCORE_CORAL_SCORING,
                 AUTO_FUNNEL_INTAKE_WAITING_ALIGN, AUTO_FUNNEL_INTAKE_WAITING_SHAKE,
                 AUTO_DESCORE_ALGAE_WAIT_INITIAL, AUTO_DESCORE_ALGAE_MOVE_BACK -> DashboardColors.autoScoring.get();

            case AUTO_SCORE_CORAL_WAIT_FINAL, AUTO_SCORE_CORAL_WAIT_ELEVATOR,
                 AUTO_DESCORE_ALGAE_WAIT_FINAL, AUTO_DESCORE_ALGAE_WAIT_AMPERAGE ->
                    autoScoreForceable ? DashboardColors.driverConfirm.get() : DashboardColors.autoScoring.get();

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

        if (inputs.funnelBeamBreakTriggered) {
            Pose3d coralInFunnel = robotPose.transformBy(new Transform3d(
                    Units.inchesToMeters(-4),
                    0,
                    Units.inchesToMeters(12),
                    new Rotation3d(0, Units.degreesToRadians(7), 0)
            ));
            Logger.recordOutput("Superstructure/CoralInFunnel", new Pose3d[]{coralInFunnel});
        } else {
            Logger.recordOutput("Superstructure/CoralInFunnel", new Pose3d[]{});
        }

        if (inputs.endEffectorBeamBreakTriggered) {
            double angle = Units.degreesToRadians(-endEffector.getAngleDegrees() - 90);
            Pose3d coralInEndEffector = robotPose.transformBy(new Transform3d(
                    Units.inchesToMeters(-8.5) + Units.inchesToMeters(6) * Math.tan(angle),
                    0,
                    Units.inchesToMeters(13.5) + elevator.getPositionMeters() + Units.inchesToMeters(4) * Math.tan(angle),
                    new Rotation3d(0, angle, 0)
            ));
            Logger.recordOutput("Superstructure/CoralInEndEffector", new Pose3d[]{coralInEndEffector});
        } else {
            Logger.recordOutput("Superstructure/CoralInEndEffector", new Pose3d[]{});
        }

        Logger.recordOutput("Superstructure/AutoScoreCoralFinalAlign", getFinalAlignPose(1, operatorDashboard.getSelectedReefZoneSide(), operatorDashboard.getSelectedLocalReefSide()));
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
        return Commands.sequence(
                backgroundCommandScheduler.waitUntilFinish(),
                cancel()
        );
    }

    private Command wrapExposedCommand(Command command) {
        // Note: if you are modifying this, there are some commands
        // that integrate ensureNotBusyAndResetGoals directly, instead
        // of using wrapExposedCommand. Make sure those are included in the changes too.
        return Commands.sequence(
                ensureNotBusyAndResetGoals(),
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
        return Commands.sequence(
                waitUntilEndEffectorTriggered(Commands.none())
                        .deadlineFor(Commands.parallel(
                                setGoal(Goal.HANDOFF),
                                endEffector.setGoal(EndEffector.RollersGoal.FUNNEL_INTAKE),
                                funnelSetGoalIntakeAlternate()
                        )),
                Commands.parallel(
                        setGoal(Goal.HOME),
                        endEffector.moveByAndWaitUntilDone(() -> Units.inchesToMeters(funnelIntakeFinalizeInches.get())),
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
                endEffector.setGoal(EndEffector.RollersGoal.EJECT), funnel.startRun(
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
                Commands.either(
                        endEffector.setGoal(EndEffector.RollersGoal.SCORE_CORAL_L1),
                        endEffector.setGoal(EndEffector.RollersGoal.SCORE_CORAL),
                        () -> elevatorGoalSupplier.get() == Elevator.Goal.SCORE_L1
                ),
                elevator.setGoal(elevatorGoalSupplier),
                waitUntilEndEffectorNotTriggered(Commands.waitSeconds(0.5))
        );

        // Wait for coral to settle
        Command finalize = Commands.either(
                Commands.waitSeconds(scoreCoralL1SettleSeconds),
                Commands.waitSeconds(scoreCoralSettleSeconds),
                () -> elevatorGoalSupplier.get() == Elevator.Goal.SCORE_L1
        );

        if (duringAuto) {
            return Commands.sequence(
                    ensureNotBusyAndResetGoals(),
                    raiseElevator,
                    waitConfirm,
                    score,
                    finalize
            );
        } else {
            return wrapExposedCommand(CommandsExt.onlyIf(
                    () -> endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                    Commands.sequence(
                            raiseElevator,
                            waitConfirm,
                            backgroundCommandScheduler.scheduleInBackground(Commands.sequence(
                                    score,
                                    finalize
                            ))
                    )
            ));
        }
    }

    public Command descoreAlgaeManual(Supplier<Elevator.Goal> elevatorGoalSupplier) {
        return wrapExposedCommand(CommandsExt.onlyIf(
                () -> !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                Commands.sequence(
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
                    Commands.sequence(
                            intake,
                            backgroundCommandScheduler.scheduleInBackground(handoffAndHome())
                    )
            ));
        } else {
            return wrapExposedCommand(CommandsExt.onlyIf(
                    () -> !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                    Commands.sequence(
                            intake,
                            backgroundCommandScheduler.scheduleInBackground(handoffAndHome())
                    )
            ));
        }
    }

    public Command autoFunnelIntake(boolean duringAuto, Station station) {
        Supplier<Pose2d> alignPoseSupplier = () -> getStationAlignPose(station);
        Command intake = Commands.race(
                waitUntilEndEffectorTriggered(Commands.idle()),
                waitUntilFunnelTriggered()
        ).deadlineFor(
                endEffector.setGoal(EndEffector.RollersGoal.FUNNEL_INTAKE),
                funnelSetGoalIntakeAlternate(),
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
                                shake()
                        )
                )
        );
        if (duringAuto) {
            return wrapExposedCommand(CommandsExt.onlyIf(
                    () -> !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                    Commands.sequence(
                            intake,
                            backgroundCommandScheduler.scheduleInBackground(handoffAndHome())
                    )
            ));
        } else {
            return wrapExposedCommand(CommandsExt.onlyIf(
                    () -> !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                    Commands.sequence(
                            intake,
                            backgroundCommandScheduler.scheduleInBackground(handoffAndHome())
                    )
            ));
        }
    }

    private boolean isAtPoseWithTolerance(Pose2d desiredPose, double toleranceXYMeters, double angularToleranceRad) {
        return isAtPoseWithTolerance(desiredPose, toleranceXYMeters, toleranceXYMeters, angularToleranceRad);
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

    public Command autoScoreCoral(
            boolean duringAuto,
            Supplier<ReefZoneSide> reefSideSupplier,
            Supplier<LocalReefSide> sideSupplier,
            Supplier<Elevator.Goal> elevatorGoalSupplier,
            BooleanSupplier forceCondition
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
        Command waitFinalAndElevator = Commands.sequence(
                Commands.parallel(
                        setGoal(Goal.AUTO_SCORE_CORAL_WAIT_FINAL),
                        endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                        elevator.setGoal(elevatorGoalSupplier),
                        Commands.waitUntil(() ->
                                isAtPoseWithTolerance(
                                        finalPoseSupplier.get(),
                                        finalAlignToleranceXYMeters,
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
                Commands.either(
                        endEffector.setGoal(EndEffector.RollersGoal.SCORE_CORAL_L1),
                        endEffector.setGoal(EndEffector.RollersGoal.SCORE_CORAL),
                        () -> elevatorGoalSupplier.get() == Elevator.Goal.SCORE_L1
                ),
                waitUntilEndEffectorNotTriggered(Commands.waitSeconds(0.5))
        );
        // Wait for coral to settle and send the elevator back down
        Command finalize = Commands.parallel(
                elevator.setGoal(() -> Elevator.Goal.STOW),
                Commands.either(
                        Commands.waitSeconds(scoreCoralL1SettleSeconds),
                        Commands.waitSeconds(scoreCoralSettleSeconds),
                        () -> elevatorGoalSupplier.get() == Elevator.Goal.SCORE_L1
                )
        );
        if (duringAuto) {
            return Commands.sequence(
                    ensureNotBusyAndResetGoals() // MUST BE CALLED AT THE START OF EVERY EXPOSED COMMAND
                            .raceWith(Commands.sequence(
                                    drive.moveTo(initialPoseSupplier)
                                            // We don't really care about position tolerances right now,
                                            // checking velocity is a good way to approximate "we're at the position we want"
                                            .until(() -> Math.abs(drive.getMeasuredChassisLinearVelocityMetersPerSec()) < finalAlignToleranceMetersPerSecond
                                                    && Math.abs(drive.getMeasuredChassisAngularVelocityRadPerSec()) < finalAlignToleranceRadPerSecond),
                                    shake()
                            )),
                    initial,
                    Commands.race(
                            drive.moveTo(finalPoseSupplier),
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
        } else
            return wrapExposedCommand(CommandsExt.onlyIf(
                    // Only run if you have coral and are in front of your reef side
                    () -> (endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get())
                            && alignable(reefSideSupplier.get(), robotState.getPose()),
                    Commands.sequence(
                            initial,
                            Commands.race(
                                    drive.moveTo(finalPoseSupplier),
                                    waitFinalAndElevator,
                                    waitForForce
                            ),
                            backgroundCommandScheduler.scheduleInBackground(Commands.race(
                                    drive.moveTo(finalPoseSupplier),
                                    score.andThen(finalize)
                            ))
                    )
            ));
    }

    public Command autoDescoreAlgae(
            Supplier<ReefZoneSide> reefSideSupplier,
            Supplier<Elevator.Goal> elevatorGoalSupplier,
            BooleanSupplier forceCondition
    ) {
        Supplier<Pose2d> poseSupplier = () -> getFinalAlignPose(1, reefSideSupplier.get(), LocalReefSide.Middle);

        Command driveTo = Commands.race(
                // Drive to position
                drive.moveTo(poseSupplier),
                Commands.sequence(
                        Commands.parallel(
                                setGoal(Goal.AUTO_DESCORE_ALGAE_WAIT_INITIAL),
                                endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                                elevator.setGoal(() -> Elevator.Goal.STOW),
                                Commands.waitUntil(() ->
                                        isAtPoseWithTolerance(
                                                poseSupplier.get(),
                                                elevatorRaiseDistanceMeters,
                                                Units.degreesToRadians(180)
                                        )
                                                && Math.abs(drive.getMeasuredChassisAngularVelocityRadPerSec()) < initialAlignToleranceRadPerSecond
                                )
                        ),
                        Commands.parallel(
                                setGoal(Goal.AUTO_DESCORE_ALGAE_WAIT_FINAL),
                                endEffector.setGoal(EndEffector.RollersGoal.DESCORE_ALGAE),
                                elevator.setGoal(elevatorGoalSupplier),
                                Commands.waitUntil(() ->
                                        isAtPoseWithTolerance(
                                                poseSupplier.get(),
                                                finalAlignToleranceXYMeters,
                                                finalAlignToleranceRad
                                        )
                                )
                        )
                )
        );

        Command waitAlgae = Commands.parallel(
                Commands.race(
                        drive.runRobotRelative(
                                () -> new ChassisSpeeds(-0.4, 0, 0)
                        ),
                        Commands.sequence(
                                Commands.waitSeconds(0.5),
                                endEffector.waitUntilDescoreAmperageTriggered()
                        )
                ),
                setGoal(Goal.AUTO_DESCORE_ALGAE_WAIT_AMPERAGE)
        );

        Command driveBack = Commands.parallel(
                drive.runRobotRelative(
                        () -> new ChassisSpeeds(0.7, 0, 0)
                ).withTimeout(0.5),
                setGoal(Goal.AUTO_DESCORE_ALGAE_MOVE_BACK)
        );

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

        return wrapExposedCommand(CommandsExt.onlyIf(
                () -> (!endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get())
                        && alignable(reefSideSupplier.get(), robotState.getPose()),
                Commands.sequence(
                        Commands.race(
                                Commands.sequence(
                                        driveTo,
                                        waitAlgae
                                ),
                                waitForForce
                        ),
                        driveBack
                )
        ));
    }
}