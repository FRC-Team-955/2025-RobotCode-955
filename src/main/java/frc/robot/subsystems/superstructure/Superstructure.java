package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.commands.CommandsExt;
import frc.lib.subsystem.CommandBasedSubsystem;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.gamePieceVision.GamePieceVision;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import lombok.RequiredArgsConstructor;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.createIO;

public class Superstructure extends CommandBasedSubsystem {


    private final RobotState robotState = RobotState.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final IntakePivot intakePivot = IntakePivot.get();
    private final IntakeRoller intakeRoller = IntakeRoller.get();
    public final Drive drive = Drive.get();
    private final SimulatedArena arena = SimulatedArena.getInstance();

    private final GamePieceVision gamePieceVision = GamePieceVision.get();


    //  private final AprilTagVision aprilTagVision = AprilTagVision.get();


    private final SuperstructureIO io = createIO();
    private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE,
        EJECT,
        INTAKE,
        AUTO_ALIGN_TO_CORAl_INTAKE,
        ROTATE_TO_CORAL,
        AUTO_SCORE_CORAL_SCORING,
        AUTO_SCORE_CORAL_WAIT_BEFORE_SCORING,
        AUTO_SCORE_CORAL_WAIT_FOR_ELEVATOR,
        AUTO_SCORE_CORAL_WAIT_FOR_ALIGN,
        AUTO_SCORE_CORAL_WAIT_UNTIL_CAN_RAISE,
        RESET_POS,
        SCORE,


    }

    private Goal goal = Goal.IDLE;


    public Command setGoal(Goal superstructureGoal,
                           Supplier<IntakePivot.IntakePivotGoal>
                                   intakePivotGoalSupplier,
                           Supplier<IntakeRoller.IntakeRollerGoal> intakeRollerGoalSupplier,
                           Supplier<OperatorDashboard.CoralScoringLevel> coralScoringLevelSupplier) {
        return runOnce(() -> {
            goal = superstructureGoal;
            intakePivot.setIntakePivotGoal(intakePivotGoalSupplier.get());
            intakeRoller.setIntakeRollerGoal(intakeRollerGoalSupplier.get());
            operatorDashboard.setSelectedCoralScoringLevel(coralScoringLevelSupplier.get());
        });


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
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Goal", goal);
    }

    public Command eject() {
//        runOnce(() -> intakeRoller.spawnCoral());
        return setGoal(
                Goal.EJECT,
                () -> IntakePivot.IntakePivotGoal.INTAKE,
                () -> IntakeRoller.IntakeRollerGoal.EJECT,
                () -> OperatorDashboard.CoralScoringLevel.EJECT
        ).andThen(runOnce(intakeRoller::spawnCoral));
    }

    public Command score() {
//        runOnce(() -> intakeRoller.spawnCoral());
        return setGoal(
                Goal.SCORE,
                () -> IntakePivot.IntakePivotGoal.STOW,
                () -> IntakeRoller.IntakeRollerGoal.IDLE,
                () -> OperatorDashboard.CoralScoringLevel.L4
        ).andThen(runOnce(intakeRoller::spawnCoral));
    }


    //
    public Command cancel() {
        return CommandsExt.eagerSequence(setGoal(
                Goal.IDLE,
                () -> IntakePivot.IntakePivotGoal.STOW,
                () -> IntakeRoller.IntakeRollerGoal.IDLE,
                () -> OperatorDashboard.get().getSelectedCoralScoringLevel()


        ).ignoringDisable(true));
    }


    public Command intake() {
        return setGoal(
                Goal.INTAKE,
                () -> IntakePivot.IntakePivotGoal.INTAKE,
                () -> IntakeRoller.IntakeRollerGoal.INTAKE,
                () -> OperatorDashboard.get().getSelectedCoralScoringLevel()
        );
    }


    public Command resetPos() {
        return
                robotState.setPose(
                        ModuleIOSim.driveSimulation::getSimulatedDriveTrainPose).alongWith(cancel());

//                .until(() ->
//                Math.abs(yaw) < 1)


    }

    public Command rotateUntilCoralVisible() {
        return
                drive.runRobotRelative(() ->
                        ChassisSpeeds.fromRobotRelativeSpeeds(
                                0.0, 0.0,
                                Math.PI, new Rotation2d(0))  // spin
                ).until(gamePieceVision::getVisible);

    }

    public Command autoIntakeCoral() {
        return rotateUntilCoralVisible()
                .andThen(autoAlignHAHA())
                .alongWith(
                        intake()
                )
                .until(intakeRoller::hasCoral); // or range sensor
    }

    public Command autoScoreCoral(
            Supplier<ReefAlign.ReefZoneSide> reefSideSupplier,
            Supplier<ReefAlign.LocalReefSide> sideSupplier,
            Supplier<OperatorDashboard.CoralScoringLevel> coralScoringLevelSupplier,
            BooleanSupplier forceCondition
    ) {
        return new AutoScoreCoral(
                reefSideSupplier,
                sideSupplier,
                coralScoringLevelSupplier,
                forceCondition).create().andThen();
    }

    public Command autoAlignHAHA() {
        return drive.runRobotRelative(() -> {
            Translation2d rel = gamePieceVision.getCoralRobotRelative();
            double yaw = gamePieceVision.getCoralYaw().getRadians();
            PIDController pidX = new PIDController(5.0, 0.0, 0.01);
            PIDController pidY = new PIDController(5.0, 0.0, 0.01);
            PIDController pidOmega = new PIDController(5.0, 0.0, 0.01);

            pidOmega.enableContinuousInput(-Math.PI, Math.PI);
            double vx = pidX.calculate(0.0, rel.getX());
            double vy = pidY.calculate(0.0, rel.getY());
            double omega = pidOmega.calculate(0.0, yaw);
            return ChassisSpeeds.fromRobotRelativeSpeeds(
                    vx, vy,
                    omega, new Rotation2d(0));
        });

//                .until(() ->
//                Math.abs(yaw) < 1)


    }
}





