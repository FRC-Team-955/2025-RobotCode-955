package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.commands.CommandsExt;
import frc.lib.subsystem.CommandBasedSubsystem;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.createIO;

public class Superstructure extends CommandBasedSubsystem {
    private final RobotState robotState = RobotState.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final SuperstructureIO io = createIO();
    private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE,
        ;
    }

    private Goal goal = Goal.IDLE;

    public Command setGoal(Goal superstructureGoal) {
        return runOnce(() -> {
            goal = superstructureGoal;
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

    private final Debouncer endEffectorBeamBreakDebouncer = new Debouncer(3 * 0.02);
    @AutoLogOutput(key = "Superstructure/EndEffectorTriggered")
    @Getter
    private boolean endEffectorTriggered = false;

    private final Debouncer funnelBeamBreakDebouncer = new Debouncer(3 * 0.02);
    @AutoLogOutput(key = "Superstructure/FunnelTriggered")
    @Getter
    private boolean funnelTriggered = false;

    private final Debouncer hasCoralDebouncer = new Debouncer(1, Debouncer.DebounceType.kFalling);
    @AutoLogOutput(key = "Superstructure/HasCoral")
    @Getter
    private boolean hasCoral = false;

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superstructure", inputs);

        // OperatorDashboard periodicBeforeCommands runs after superstructure
//        throw new RuntimeException("TODO just get the closest side at the start of the command");
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
    }

    public Command cancel() {
        return CommandsExt.eagerSequence(
                setGoal(
                        Goal.IDLE
                )
        ).ignoringDisable(true);
    }
}