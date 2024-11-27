package frc.robot.subsystems.shooter;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.Util;
import frc.robot.dashboard.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.GoalBasedCommandRunner;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class Shooter extends SubsystemBase {
    ////////////////////// GOAL SETPOINTS - HOVER //////////////////////
    private static final TuningDashboardAngle hoverPivot = new TuningDashboardAngle(
            DashboardSubsystem.SHOOTER, "Hover Pivot",
            Degrees.of(-90)
    );

    ////////////////////// GOAL SETPOINTS - WAIT FOR INTAKE //////////////////////
    private static final TuningDashboardAngle waitForIntakePivot = new TuningDashboardAngle(
            DashboardSubsystem.SHOOTER, "Wait For Intake Pivot",
            Degrees.of(-30)
    );

    ////////////////////// GOAL SETPOINTS - EJECT //////////////////////
    private static final TuningDashboardAngle ejectPivot = new TuningDashboardAngle(
            DashboardSubsystem.SHOOTER, "Eject Pivot",
            Degrees.of(-60)
    );
    private static final TuningDashboardAnglularVelocityRPM ejectFlywheels = new TuningDashboardAnglularVelocityRPM(
            DashboardSubsystem.SHOOTER, "Eject Flywheels",
            RPM.of(2000)
    );

    ////////////////////// GOAL SETPOINTS - SHOOT CONFIGURABLE //////////////////////
    private static final TuningDashboardAngle shootConfigurablePivot = new TuningDashboardAngle(
            DashboardSubsystem.SHOOTER, "Shoot Configurable Pivot",
            Degrees.zero()
    );
    private static final TuningDashboardAnglularVelocityRPM shootConfigurableFlywheels = new TuningDashboardAnglularVelocityRPM(
            DashboardSubsystem.SHOOTER, "Shoot Configurable Flywheels",
            RPM.zero()
    );

    ////////////////////// GOAL SETPOINTS - SHOOT PASSING ////////////////////////

    private static final TuningDashboardAngle shootPassingPivot = new TuningDashboardAngle(
            DashboardSubsystem.SHOOTER, "Shoot Passing Pivot",
            Degrees.of(-45)
    );

    private static final TuningDashboardAnglularVelocityRPM shootPassingFlywheels = new TuningDashboardAnglularVelocityRPM(
            DashboardSubsystem.SHOOTER, "Shoot Passing Flywheels",
            RPM.of(4000)
    );

    ////////////////////// GOAL SETPOINTS - SHOOT SUBWOOFER //////////////////////
    private static final TuningDashboardAngle shootSubwooferPivot = new TuningDashboardAngle(
            DashboardSubsystem.SHOOTER, "Shoot Subwoofer Pivot",
            Degrees.of(-50)
    );
    private static final TuningDashboardAnglularVelocityRPM shootSubwooferFlywheels = new TuningDashboardAnglularVelocityRPM(
            DashboardSubsystem.SHOOTER, "Shoot Subwoofer Flywheels",
            RPM.of(3500)
    );

    ////////////////////// GOAL SETPOINTS - AMP //////////////////////
    private static final TuningDashboardAngle ampPivot = new TuningDashboardAngle(
            DashboardSubsystem.SHOOTER, "Amp Pivot",
            Degrees.of(25)
    );
    private static final TuningDashboardAnglularVelocityRPM ampFlywheels = new TuningDashboardAnglularVelocityRPM(
            DashboardSubsystem.SHOOTER, "Amp Flywheels",
            RPM.of(2000)
    );

    ////////////////////// GOAL SETPOINTS - HANDOFF //////////////////////
    private static final TuningDashboardAngle handoffPivot = new TuningDashboardAngle(
            DashboardSubsystem.SHOOTER, "Handoff Pivot",
            Degrees.of(-50)
    );
    private static final TuningDashboardAnglularVelocityRPM handoffFeed = new TuningDashboardAnglularVelocityRPM(
            DashboardSubsystem.SHOOTER, "Handoff Feed",
            RPM.of(500)
    );

    // trick Java into letting us use an enum before it is defined
    private static final Goal GOAL_WAIT_FOR_INTAKE = Goal.WAIT_FOR_INTAKE;

    public enum Goal {
        CHARACTERIZATION(() -> null, () -> null, () -> null),
        ZERO(() -> null, RPM::zero, () -> FeedSetpoint.velocity(RPM.zero())),
        HOVER(
                hoverPivot::get,
                RPM::zero,
                () -> FeedSetpoint.velocity(RPM.zero()),
                () -> Intake.get().pivotClearOfShooter() || get().atGoal() ? Optional.empty() : Optional.of(GOAL_WAIT_FOR_INTAKE)
        ),
        WAIT_FOR_INTAKE(
                waitForIntakePivot::get,
                RPM::zero,
                () -> FeedSetpoint.velocity(RPM.zero()),
                () -> Intake.get().pivotClearOfShooter() ? Optional.of(Goal.HOVER) : Optional.empty()
        ),
        SHOOT_CALCULATED_SPINUP(
                () -> get().pivotSetpoint,
                () -> ShooterRegression.getSpeed(Feet.zero()/*RobotState.get().getDistanceToSpeaker()*/),
                () -> FeedSetpoint.velocity(RPM.zero())
        ),
        SHOOT_CALCULATED(
                () -> ShooterRegression.getAngle(Feet.zero()/*RobotState.get().getDistanceToSpeaker()*/).plus(shootingSkew.get()),
                () -> ShooterRegression.getSpeed(Feet.zero()/*RobotState.get().getDistanceToSpeaker()*/),
                FeedSetpoint::shoot
        ),
        SHOOT_CONFIGURABLE(
                shootConfigurablePivot::get,
                shootConfigurableFlywheels::get,
                FeedSetpoint::shoot
        ),
        SHOOT_PASSING(
                shootPassingPivot::get,
                shootPassingFlywheels::get,
                FeedSetpoint::shoot
        ),
        SHOOT_SUBWOOFER(
                () -> shootSubwooferPivot.get().plus(shootingSkew.get()),
                shootSubwooferFlywheels::get,
                FeedSetpoint::shoot
        ),
        AMP(ampPivot::get, ampFlywheels::get, FeedSetpoint::shoot),
        EJECT(ejectPivot::get, ejectFlywheels::get, FeedSetpoint::shoot),

        HANDOFF_WAIT_FOR_INTAKE(
                waitForIntakePivot::get,
                () -> DriverStation.isAutonomous()
                        ? ShooterRegression.getSpeed(Feet.zero()/*RobotState.get().getDistanceToSpeaker()*/)
                        : RPM.zero(),
                () -> FeedSetpoint.velocity(RPM.zero())
        ),
        HANDOFF_READY(
                handoffPivot::get,
                () -> DriverStation.isAutonomous()
                        ? ShooterRegression.getSpeed(Feet.zero()/*RobotState.get().getDistanceToSpeaker()*/)
                        : RPM.zero(),
                () -> FeedSetpoint.velocity(RPM.zero())
        ),
        HANDOFF_FEED(
                handoffPivot::get,
                () -> DriverStation.isAutonomous()
                        ? ShooterRegression.getSpeed(Feet.zero()/*RobotState.get().getDistanceToSpeaker()*/)
                        : RPM.zero(),
                () -> FeedSetpoint.velocity(handoffFeed.get())
        );

        public static final Goal DEFAULT = Goal.HOVER;

        public final Supplier<Measure<Angle>> pivotSetpoint;
        public final Supplier<Measure<Velocity<Angle>>> flywheelsSetpoint;
        public final Supplier<FeedSetpoint> feedSetpoint;
        public final Supplier<Optional<Goal>> goalChange;

        Goal(
                Supplier<Measure<Angle>> pivotSetpoint,
                Supplier<Measure<Velocity<Angle>>> flywheelsSetpoint,
                Supplier<FeedSetpoint> feedCommand
        ) {
            this.pivotSetpoint = pivotSetpoint;
            this.flywheelsSetpoint = flywheelsSetpoint;
            this.feedSetpoint = feedCommand;
            this.goalChange = Optional::empty;
        }

        Goal(
                Supplier<Measure<Angle>> pivotSetpoint,
                Supplier<Measure<Velocity<Angle>>> flywheelsSetpoint,
                Supplier<FeedSetpoint> feedCommand,
                Supplier<Optional<Goal>> goalChange
        ) {
            this.pivotSetpoint = pivotSetpoint;
            this.flywheelsSetpoint = flywheelsSetpoint;
            this.feedSetpoint = feedCommand;
            this.goalChange = goalChange;
        }

        public static class FeedSetpoint {
            private final Type type;
            private final Measure<Velocity<Angle>> velocity;

            private enum Type {
                Velocity,
                Shoot
            }

            private FeedSetpoint(Type type, Measure<Velocity<Angle>> velocity) {
                this.type = type;
                this.velocity = velocity;
            }

            public static FeedSetpoint velocity(Measure<Velocity<Angle>> velocity) {
                return new FeedSetpoint(Type.Velocity, velocity);
            }

            public static FeedSetpoint shoot() {
                return new FeedSetpoint(Type.Shoot, null);
            }

            public boolean isShoot() {
                return type == Type.Shoot;
            }

            public void ifVelocity(Consumer<Measure<Velocity<Angle>>> velocityConsumer, Runnable elseRun) {
                if (type == Type.Velocity)
                    velocityConsumer.accept(velocity);
                else
                    elseRun.run();
            }
        }
    }

    ////////////////////// CONSTANTS //////////////////////

    protected static final double PIVOT_GEAR_RATIO = 40;
    protected static final Measure<Angle> PIVOT_INITIAL_POSITION = Degrees.of(-90);
    protected static final double FEED_GEAR_RATIO = 3;
    protected static final double FLYWHEEL_GEAR_RATIO = 1 / 2.0;
    private final Debouncer hasNoteDebouncer = new Debouncer(0.01);

    ////////////////////// GENERAL DASHBOARD VALUES //////////////////////

    private static final DashboardAngle shootingSkew = new DashboardAngle(
            DashboardSubsystem.SHOOTER, "Shooting Skew",
            Degrees.of(3)
    );

    private static final TuningDashboardAngle pivotFFOffset = new TuningDashboardAngle(
            DashboardSubsystem.SHOOTER, "Pivot FF Offset",
            Degrees.of(0)
    );
    private static final TuningDashboardAngle pivotSetpointToleranceShooting = new TuningDashboardAngle(
            DashboardSubsystem.SHOOTER, "Pivot Tolerance - Shooting",
            Degrees.of(2.5)
    );
    private static final TuningDashboardAngle pivotSetpointToleranceNotShooting = new TuningDashboardAngle(
            DashboardSubsystem.SHOOTER, "Pivot Tolerance - Not Shooting",
            Degrees.of(5)
    );

    private static final TuningDashboardAnglularVelocityRPM feedSetpointTolerance = new TuningDashboardAnglularVelocityRPM(
            DashboardSubsystem.SHOOTER, "Feed Tolerance",
            RPM.of(10)
    );
    private static final TuningDashboardAnglularVelocityRPM flywheelSetpointTolerance = new TuningDashboardAnglularVelocityRPM(
            DashboardSubsystem.SHOOTER, "Flywheel Tolerance",
            RPM.of(100)
    );

    private static final DashboardButton pivotZero = new DashboardButton(DashboardSubsystem.SHOOTER, "Pivot Zero");
    private static final TuningDashboardNumber pivotZeroUpVoltage = new TuningDashboardNumber(
            DashboardSubsystem.SHOOTER, "Pivot Zero Up Voltage",
            5
    );
    private static final TuningDashboardNumber pivotZeroDownVoltage = new TuningDashboardNumber(
            DashboardSubsystem.SHOOTER, "Pivot Zero Down Voltage",
            -3
    );

    ////////////////////// IO //////////////////////
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    ////////////////////// PIVOT //////////////////////
    private final TuningDashboardArmFeedforward pivotFeedforward = new TuningDashboardArmFeedforward(
            DashboardSubsystem.SHOOTER, "Pivot FF",
            /*Constants.isReal
                    ? new ArmFeedforward(0.574, 1.0, 0.65051, 0.21235)
                    :*/ new ArmFeedforward(0, 0.4, 0)
    );
    private final TuningDashboardPIDConstants pivotPIDShooting = new TuningDashboardPIDConstants(
            DashboardSubsystem.SHOOTER, "Pivot PID Shooting",
            /*Constants.isReal
                    ? new PIDConstants(0.2, 0.001)
                    :*/ new PIDConstants(5, 0)
    );
    private final TuningDashboardPIDConstants pivotPIDNotShooting = new TuningDashboardPIDConstants(
            DashboardSubsystem.SHOOTER, "Pivot PID Not Shooting",
            /*Constants.isReal
                    ? new PIDConstants(0.075)
                    :*/ new PIDConstants(5, 0)
    );
    private Measure<Angle> pivotSetpoint = PIVOT_INITIAL_POSITION;
    public final SysIdRoutine pivotSysId;
    private Goal pivotPIDGoalConfigured = Goal.DEFAULT;

    ////////////////////// FEED //////////////////////
    private final TuningDashboardSimpleFeedforward feedFeedforward = new TuningDashboardSimpleFeedforward(
            DashboardSubsystem.SHOOTER, "Feed FF",
            /*Constants.isReal
                    ? new SimpleMotorFeedforward(0.23233, 0.060739, 0.006805)
                    :*/ new SimpleMotorFeedforward(0, 0.058)
    );
    private final TuningDashboardPIDConstants feedPID = new TuningDashboardPIDConstants(
            DashboardSubsystem.SHOOTER, "Feed PID",
            /*Constants.isReal
                    ? new PIDConstants(0.0001, 0.0001, 0)
                    :*/ new PIDConstants(0.1, 0)
    );
    private Measure<Velocity<Angle>> feedSetpoint = null;
    public final SysIdRoutine feedSysId;

    ////////////////////// FLYWHEELS //////////////////////
    private final TuningDashboardSimpleFeedforward flywheelTopFeedforward = new TuningDashboardSimpleFeedforward(
            DashboardSubsystem.SHOOTER, "Flywheel Top FF",
            /*Constants.isReal
                    ? new SimpleMotorFeedforward(0.23795, 0.011308, 0.0069895)
                    :*/ new SimpleMotorFeedforward(0, 0.01)
    );
    private final TuningDashboardSimpleFeedforward flywheelBottomFeedforward = new TuningDashboardSimpleFeedforward(
            DashboardSubsystem.SHOOTER, "Flywheel Bottom FF",
            /*Constants.isReal
                    ? new SimpleMotorFeedforward(0.23279, 0.0113005, 0.0064997)
                    :*/ flywheelTopFeedforward.get()
    );
    private final TuningDashboardPIDConstants flywheelTopPID = new TuningDashboardPIDConstants(
            DashboardSubsystem.SHOOTER, "Flywheel Top PID",
            /*Constants.isReal
                    ? new PIDConstants(0.0001, 0.01, 0)
                    :*/ new PIDConstants(0.2, 0));
    private final TuningDashboardPIDConstants flywheelBottomPID = new TuningDashboardPIDConstants(
            DashboardSubsystem.SHOOTER, "Flywheel Bottom PID",
            /*Constants.isReal
                    ? new PIDConstants(0.0001, 0.01, 0)
                    :*/ flywheelTopPID.get()
    );
    private Measure<Velocity<Angle>> flywheelsSetpoint = null;
    public final SysIdRoutine flywheelsSysId;

    ////////////////////// GOAL STUFF //////////////////////
    @Getter
    private Goal goal = Goal.DEFAULT;
    private boolean goalFinished = false;

    private Command goal(Goal newGoal) {
        return new FunctionalCommand(
                () -> {
                    goalFinished = false;
                    goal = newGoal;
                    processGoal();
                },
                () -> {
                },
                (interrupted) -> {
                    goalFinished = false;
                    goal = Goal.DEFAULT;
                    processGoal();
                },
                // end if the goal finished
                () -> goalFinished,
                this
        );
    }

    private void processGoal() {
        if (goal.goalChange != null)
            goal.goalChange.get().ifPresent((newGoal) -> goal = newGoal);
        Logger.recordOutput("Shooter/Goal", goal);
        pivotSetpoint = goal.pivotSetpoint.get();
        flywheelsSetpoint = goal.flywheelsSetpoint.get();
        var feedSetpoint = goal.feedSetpoint.get();
        if (feedSetpoint != null)
            feedSetpoint.ifVelocity(
                    (velocity) -> this.feedSetpoint = velocity,
                    () -> this.feedSetpoint = null
            );
        else
            this.feedSetpoint = null;
    }

    public boolean atGoal() {
        var pivotTolerance = switch (goal) {
            case SHOOT_CALCULATED, SHOOT_CONFIGURABLE, SHOOT_PASSING -> pivotSetpointToleranceShooting.get();
            default -> pivotSetpointToleranceNotShooting.get();
        };
        var pivotAtSetpoint = pivotSetpoint == null || Math.abs(inputs.pivotPositionRad - pivotSetpoint.in(Radians)) <= pivotTolerance.in(Radians);

        var feedAtSetpoint = feedSetpoint == null || Math.abs(inputs.feedVelocityRadPerSec - feedSetpoint.in(RadiansPerSecond)) <= feedSetpointTolerance.get().in(RadiansPerSecond);

        var flywheelsTolerance = flywheelSetpointTolerance.get().in(RadiansPerSecond);
        var flywheelsAtSetpoint =
                flywheelsSetpoint == null ||
                        switch (goal) {
                            case SHOOT_CALCULATED_SPINUP, HANDOFF_FEED, HANDOFF_READY, HANDOFF_WAIT_FOR_INTAKE -> true;
                            default -> false;
                        } ||
                        (Math.abs(inputs.flywheelTopVelocityRadPerSec - flywheelsSetpoint.in(RadiansPerSecond)) <= flywheelsTolerance &&
                                Math.abs(inputs.flywheelBottomVelocityRadPerSec - flywheelsSetpoint.in(RadiansPerSecond)) <= flywheelsTolerance);

        return pivotAtSetpoint && feedAtSetpoint && flywheelsAtSetpoint;
    }

    ////////////////////// INSTANCE STUFF //////////////////////
    private static Shooter instance;

    public static Shooter get() {
        if (instance == null)
            synchronized (Shooter.class) {
                instance = new Shooter();
            }

        return instance;
    }

    ////////////////////// CONSTRUCTOR //////////////////////
    private Shooter() {
        ////////////////////// IO CREATION //////////////////////
        io = new ShooterIO();/*switch (Constants.mode) {
            case REAL -> new ShooterIOSparkMaxBeamBreak(
                    6,
                    7,
                    9,
                    10,
                    8
            );
            case SIM -> new ShooterIOSim(
                    DCMotor.getNEO(1),
                    0.4,
                    0.083,
                    DCMotor.getNEO(1),
                    DCMotor.getNEO(1),
                    DCMotor.getNEO(1)
            );
            case REPLAY -> new ShooterIO();
        };*/

        ////////////////////// IO CONFIGURATION //////////////////////

        io.pivotConfigurePID(pivotPIDNotShooting.get());
        io.pivotSetPosition(PIVOT_INITIAL_POSITION.in(Radians));

        io.feedConfigurePID(feedPID.get());
        io.flywheelsTopConfigurePID(flywheelTopPID.get());
        io.flywheelsBottomConfigurePID(flywheelBottomPID.get());

        ////////////////////// TRIGGERS //////////////////////

        var feedCommandRunner = new GoalBasedCommandRunner<>("ShooterFeed", () -> goal);
        new Trigger(() -> {
            var feedSetpoint = goal.feedSetpoint.get();
            if (feedSetpoint != null)
                return feedSetpoint.isShoot() && atGoal();
            return false;
        }).onTrue(
                feedCommandRunner
                        .startEnd(
                                () -> io.feedSetVoltage(12),
                                () -> {
                                    io.feedSetVoltage(0);
                                    goalFinished = true;
                                }
                        )
                        .withTimeout(0.8)
                        .withName("Shooter Feed Shoot")
        );

        pivotZero.trigger().toggleOnTrue(zero());

        ////////////////////// SYSID //////////////////////

        pivotSysId = Util.sysIdRoutine(
                "Shooter/Pivot",
                (voltage) -> io.pivotSetVoltage(voltage.in(Volts)),
                () -> goal = Goal.CHARACTERIZATION,
                () -> goal = Goal.DEFAULT,
                this
        );

        feedSysId = Util.sysIdRoutine(
                "Shooter/Feed",
                (voltage) -> io.feedSetVoltage(voltage.in(Volts)),
                () -> goal = Goal.CHARACTERIZATION,
                () -> goal = Goal.DEFAULT,
                this
        );

        flywheelsSysId = Util.sysIdRoutine(
                "Shooter/Flywheels",
                (voltage) -> io.flywheelsSetVoltage(voltage.in(Volts)),
                () -> goal = Goal.CHARACTERIZATION,
                () -> goal = Goal.DEFAULT,
                this
        );
    }

    ////////////////////// PERIODIC //////////////////////
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Shooter", inputs);

        processGoal();

        ////////////////////// PID MANAGEMENT //////////////////////
        var pivotPID = switch (goal) {
            case SHOOT_CALCULATED, SHOOT_CONFIGURABLE, SHOOT_SUBWOOFER, AMP, SHOOT_PASSING -> pivotPIDShooting;
            default -> pivotPIDNotShooting;
        };
        if (pivotPIDGoalConfigured != goal) {
            io.pivotConfigurePID(pivotPID.get());
            pivotPIDGoalConfigured = goal;
        } else {
            pivotPID.ifChanged(io::pivotConfigurePID);
        }
        feedPID.ifChanged(io::feedConfigurePID);
        flywheelTopPID.ifChanged(io::flywheelsTopConfigurePID);
        flywheelBottomPID.ifChanged(io::flywheelsBottomConfigurePID);

        ////////////////////// PIVOT //////////////////////
        Logger.recordOutput("Shooter/Pivot/ClosedLoop", pivotSetpoint != null);
        if (pivotSetpoint != null) {
            Logger.recordOutput("Shooter/Pivot/Setpoint", pivotSetpoint);

            if (DriverStation.isEnabled()) {
                var pivotSetpointRad = pivotSetpoint.plus(pivotFFOffset.get()).in(Radians);
                var ffVolts = pivotFeedforward.get().calculate(pivotSetpointRad, 0);
                Logger.recordOutput("Shooter/Pivot/FFVolts", ffVolts);
                io.pivotSetSetpoint(pivotSetpointRad, ffVolts);
            }
        }

        ////////////////////// FEED //////////////////////
        Logger.recordOutput("Shooter/Feed/ClosedLoop", feedSetpoint != null);
        if (feedSetpoint != null) {
            Logger.recordOutput("Shooter/Feed/Setpoint", feedSetpoint);

            if (DriverStation.isEnabled()) {
                var feedSetpointRadPerSec = feedSetpoint.in(RadiansPerSecond);
                var ffVolts = feedFeedforward.get().calculate(feedSetpointRadPerSec, 0);
                Logger.recordOutput("Shooter/Feed/FFVolts", ffVolts);
                io.feedSetSetpoint(feedSetpointRadPerSec, ffVolts);
            }
        }

        ////////////////////// FLYWHEELS //////////////////////
        Logger.recordOutput("Shooter/Flywheels/ClosedLoop", flywheelsSetpoint != null);
        if (flywheelsSetpoint != null) {
            Logger.recordOutput("Shooter/Flywheels/Setpoint", flywheelsSetpoint);

            if (DriverStation.isEnabled()) {
                var flywheelsSetpointRadPerSec = flywheelsSetpoint.in(RadiansPerSecond);
                var ffTopVolts = flywheelTopFeedforward.get().calculate(flywheelsSetpointRadPerSec, 0);
                var ffBottomVolts = flywheelBottomFeedforward.get().calculate(flywheelsSetpointRadPerSec, 0);
                Logger.recordOutput("Shooter/Flywheels/FFTopVolts", ffTopVolts);
                Logger.recordOutput("Shooter/Flywheels/FFBottomVolts", ffBottomVolts);
                io.flywheelsSetSetpoint(flywheelsSetpointRadPerSec, ffTopVolts, ffBottomVolts);
            }
        }

//        if (Constants.isSim) {
        Logger.recordOutput("Shooter/Component", new Pose3d(new Translation3d(-0.27, 0, 0.59), new Rotation3d(0, -inputs.pivotPositionRad, 0)));
//        }
    }

    ////////////////////// COMMANDS //////////////////////

    public Command zero() {
        return goal(Goal.ZERO)
                .raceWith(Commands.sequence(
                        Commands.either(
                                Commands.sequence(
                                        Commands.startEnd(
                                                () -> io.pivotSetVoltage(pivotZeroUpVoltage.getRaw()),
                                                () -> io.pivotSetVoltage(0)
                                        ).withTimeout(0.25),
                                        Commands.startEnd(
                                                () -> io.pivotSetVoltage(pivotZeroDownVoltage.getRaw()),
                                                () -> io.pivotSetVoltage(0)
                                        ).withTimeout(0.75),
                                        Commands.waitSeconds(0.25)
                                ),
                                Commands.none(),
                                DriverStation::isEnabled
                        ),
                        Commands.runOnce(() -> io.pivotSetPosition(PIVOT_INITIAL_POSITION.in(Radians)))
                ))
                .ignoringDisable(true)
                .withName("Shooter Zero");
    }

    public Command handoffWaitForIntake() {
        return goal(Goal.HANDOFF_WAIT_FOR_INTAKE).withName("Shooter Handoff Wait For Intake");
    }

    public Command handoffReady() {
        return goal(Goal.HANDOFF_READY).withName("Shooter Handoff Ready");
    }

    public Command handoffFeed() {
        return goal(Goal.HANDOFF_FEED).withName("Shooter Handoff Feed");
    }

    public Command shootSubwoofer() {
        return goal(Goal.SHOOT_SUBWOOFER).withName("Shooter Shoot Subwoofer");
    }

    public Command shootCalculatedSpinup() {
        return goal(Goal.SHOOT_CALCULATED_SPINUP).withName("Shooter Shoot Calculated Spinup");
    }

    public Command shootCalculated() {
        return goal(Goal.SHOOT_CALCULATED).withName("Shooter Shoot Calculated");
    }

    public Command shootConfigurable() {
        return goal(Goal.SHOOT_CONFIGURABLE).withName("Shooter Shoot Configurable");
    }

    public Command shootPassing() {
        return goal(Goal.SHOOT_PASSING).withName("Shooter Shoot Passing");
    }

    public Command amp() {
        return goal(Goal.AMP).withName("Shooter Amp");
    }

    public Command eject() {
        return goal(Goal.EJECT).withName("Shooter Eject");
    }

    ////////////////////// MISC FUNCTIONS //////////////////////

    @AutoLogOutput
    public boolean hasNoteDebounced() {
        return hasNoteDebouncer.calculate(inputs.hasNote);
    }

    public void shootCalculatedSpinupInBackground() {
        goal = Goal.SHOOT_CALCULATED_SPINUP;
        processGoal();
    }
}
