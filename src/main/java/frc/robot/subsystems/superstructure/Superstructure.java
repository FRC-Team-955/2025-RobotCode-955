package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.coralintake.CoralIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.util.SubsystemBaseExt;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBaseExt {
    private final RobotState robotState = RobotState.get();
    private final Drive drive = Drive.get();
    private final CoralIntake coralIntake = CoralIntake.get();
    private final Indexer indexer = Indexer.get();
    private final Elevator elevator = Elevator.get();
    private final EndEffector endEffector = EndEffector.get();

    public enum Goal {
        CHARACTERIZATION,
        IDLE,
        INTAKE_CORAL,
        SCORE_CORAL_L1,
        SCORE_CORAL_L2,
        SCORE_CORAL_L3,
        SCORE_CORAL_L4
    }

    private final SuperstructureIO io = SuperstructureConstants.io;
    private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

    @Getter
    private Goal goal = Goal.IDLE;

    private Command withGoal(Goal newGoal, Command command) {
        return new WrapperCommand(command) {
            @Override
            public void initialize() {
                goal = newGoal;
                super.initialize();
            }
        };
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

    }

    public Command idle() {
        return withGoal(
                Goal.IDLE,
                Commands.sequence(
                        Commands.parallel(
                                coralIntake.setGoals(CoralIntake.PivotGoal.STOW, CoralIntake.RollersGoal.IDLE),
                                elevator.setGoal(Elevator.Goal.STOW),
                                endEffector.setGoal(EndEffector.RollersGoal.IDLE),
                                indexer.setGoal(Indexer.RollersGoal.IDLE)
                        ),
                        Commands.idle(this) // other requirements given implicitly in .parallel
                )
        );
    }

    public Command intake() {
        // TODO: after we have sensed that coral is in the system, prevent driver from messing things up - making the rest of the command uncancelable
        return null;
    }
}