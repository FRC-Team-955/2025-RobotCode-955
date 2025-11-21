package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.lib.subsystem.CommandBasedSubsystem;
import frc.robot.OperatorDashboard;
import frc.robot.RobotMechanism;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;
import frc.robot.subsystems.gamepiecevision.GamePieceVisionIO;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.util.commands.CommandsExt;
import frc.robot.util.subsystem.SubsystemBaseExt;
import lombok.Getter;
import org.ironmaple.simulation.IntakeSimulation;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.createIO;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.intakeRangeTriggerMeters;

public class Superstructure extends CommandBasedSubsystem {
    private final RobotState robotState = RobotState.get();
    private final RobotMechanism robotMechanism = RobotMechanism.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private final IntakePivot intakePivot = IntakePivot.get();
    private final IntakeRollers intakeRollers = IntakeRollers.get();
    private final Drive drive = Drive.get();

    private final SuperstructureIO io = createIO();
    private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

    public enum Goal {
        IDLE,

        INTAKE_PIVOT_STOW,
        INTAKE_PIVOT_INTAKE,
        INTAKE_PIVOT_IDLE,
        INTAKE_ROLLER_WAIT_PIVOT,
        INTAKE_ROLLER_IDLE,
        INTAKE_ROLLER_INTAKE,
        HANDOFF_WAIT,
        HANDOFF_HANDING_OFF,

        MOVING_TO_CORAL,

        EJECT,
    }

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

    private Superstructure() {}

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superstructure", inputs);
    }

    @Override
    public void periodicAfterCommands() {
    Logger.recordOutput("Superstructure/Goal", goal);
    Logger.recordOutput("Superstructure/IntakeRangeTriggered", intakeRangeTriggered());
    }

    private boolean intakeRangeTriggered() {
        return inputs.intakeRangeMeters <= intakeRangeTriggerMeters;
    }

    public Command waitUntilIntakeTriggered() {
        return Commands.waitUntil(this::intakeRangeTriggered);
    }


    public Command idle() {
        return setGoal(Goal.IDLE).andThen(Commands.idle());
    }

    public Command intakeIdle() {
        return intakePivot.setGoals(IntakePivot.IntakePivotGoal.STOW)
                .alongWith(intakeRollers.setGoals(IntakeRollers.IntakeRollersGoal.IDLE))
                .andThen(Commands.idle());
    }

    public Command intakeCoral() {
        return Commands.sequence(
                Commands.parallel(
                        setGoal(Goal.INTAKE_ROLLER_WAIT_PIVOT),
                        intakePivot.setGoals(IntakePivot.IntakePivotGoal.INTAKE),
                        intakeRollers.setGoals(IntakeRollers.IntakeRollersGoal.IDLE)
                ),
                Commands.parallel(
                        setGoal(Goal.INTAKE_ROLLER_INTAKE),
                        intakePivot.setGoals(IntakePivot.IntakePivotGoal.INTAKE),
                        intakeRollers.setGoals(IntakeRollers.IntakeRollersGoal.INTAKE)
                ),
                waitUntilIntakeTriggered(),
                Commands.sequence(
                        Commands.parallel(
                                setGoal(Goal.INTAKE_PIVOT_STOW),
                                intakePivot.setGoals(IntakePivot.IntakePivotGoal.STOW),
                                intakeRollers.setGoals(IntakeRollers.IntakeRollersGoal.IDLE)
                        )
                )
        );
    }


    public Command moveToCoralSim() {
            return Commands.sequence(
                    Commands.waitUntil(() -> {
                        Pose2d coral = GamePieceVision.get().getCoralPos();
                        Pose2d robot = ModuleIOSim.driveSimulation.getSimulatedDriveTrainPose();
                        return robot.getTranslation().getDistance(coral.getTranslation()) < 1.3;
                    }).raceWith(
                            drive.moveTo(this::getCoralApproachPoseSim, false)
                    ),
                    intakeCoral()
            );
    }

    public Pose2d getCoralApproachPoseSim() {
        Pose2d coral = GamePieceVision.get().getCoralPos();
        Pose2d robot = ModuleIOSim.driveSimulation.getSimulatedDriveTrainPose();
        double standoffDistance = 0.5;

        Rotation2d heading = new Rotation2d(
                Math.atan2(coral.getY() - robot.getY(), coral.getX() - robot.getX())
        ).rotateBy(Rotation2d.fromDegrees(180));

        Translation2d offset = new Translation2d(-standoffDistance, heading);

        return new Pose2d(coral.getX() + offset.getX(), coral.getY() + offset.getY(), heading);
    }

    public Command eject() {
        return Commands.parallel(
                setGoal(Goal.EJECT),
                Commands.idle()
        );
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
}
