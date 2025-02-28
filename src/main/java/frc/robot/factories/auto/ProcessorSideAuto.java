package frc.robot.factories.auto;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.OperatorDashboard.LocalReefSide;
import frc.robot.OperatorDashboard.ReefZoneSide;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.commands.CommandsExt;

public class ProcessorSideAuto {
    public static Command get(AutoRoutine routine) {
        final var superstructure = Superstructure.get();

        final var twoOclockLeftScoreTraj = routine.trajectory("Processor Side", 0);
        final var twoOclockRightStationTraj = routine.trajectory("Processor Side", 1);
        final var twoOclockRightScoreTraj = routine.trajectory("Processor Side", 2);
        final var fourOclockLeftStationTraj = routine.trajectory("Processor Side", 3);
        final var fourOclockLeftScoreTraj = routine.trajectory("Processor Side", 4);
        final var fourOclockRightStationTraj = routine.trajectory("Processor Side", 5);
        final var fourOclockRightScoreTraj = routine.trajectory("Processor Side", 6);

        var ref = new Object() {
            boolean isFinished = false;
        };

        routine.active().onTrue(
                Commands.sequence(
                        twoOclockLeftScoreTraj.resetOdometry(),
                        twoOclockLeftScoreTraj.cmd()
                )
        );

        twoOclockLeftScoreTraj.atTime("score").onTrue(Commands.sequence(
                superstructure.autoAlignAndScore(
                        true,
                        () -> ReefZoneSide.RightBack,
                        () -> LocalReefSide.Right,
                        () -> Elevator.Goal.SCORE_L4,
                        () -> true,
                        () -> false
                ),
                CommandsExt.schedule(twoOclockRightStationTraj.cmd()) // schedule so subsystems run their default commands and so the command doesn't cancel itself
        ));

        twoOclockRightStationTraj.atTime("intake").onTrue(Commands.sequence(
                superstructure.funnelIntakeWithAutoAlign(true),
                twoOclockRightScoreTraj.cmd()
        ));
        twoOclockRightScoreTraj.atTime("score").onTrue(Commands.sequence(
                superstructure.autoAlignAndScore(
                        true,
                        () -> ReefZoneSide.RightBack,
                        () -> LocalReefSide.Left,
                        () -> Elevator.Goal.SCORE_L4,
                        () -> true,
                        () -> false
                ),
                CommandsExt.schedule(fourOclockLeftStationTraj.cmd()) // schedule so subsystems run their default commands and so the command doesn't cancel itself
        ));

        fourOclockLeftStationTraj.atTime("intake").onTrue(Commands.sequence(
                superstructure.funnelIntakeWithAutoAlign(true),
                fourOclockLeftScoreTraj.cmd()
        ));
        fourOclockLeftScoreTraj.atTime("score").onTrue(Commands.sequence(
                superstructure.autoAlignAndScore(
                        true,
                        () -> ReefZoneSide.RightFront,
                        () -> LocalReefSide.Right,
                        () -> Elevator.Goal.SCORE_L4,
                        () -> true,
                        () -> false

                ),
                Commands.runOnce(() -> ref.isFinished = true)
//                CommandsExt.schedule(fourOclockRightStationTraj.cmd().alongWith(superstructure.funnelIntake(true))) // schedule so subsystems run their default commands and so the command doesn't cancel itself
        ));

        fourOclockRightStationTraj.atTime("intake").onTrue(Commands.sequence(
                superstructure.funnelIntakeWithAutoAlign(true),
                fourOclockRightScoreTraj.cmd()
        ));
        fourOclockRightScoreTraj.atTime("score").onTrue(Commands.sequence(
                superstructure.autoAlignAndScore(
                        true,
                        () -> ReefZoneSide.RightFront,
                        () -> LocalReefSide.Left,
                        () -> Elevator.Goal.SCORE_L4,
                        () -> true,
                        () -> false
                ),
                Commands.runOnce(() -> ref.isFinished = true)
        ));

        return routine.cmd(() -> ref.isFinished).beforeStarting(() -> ref.isFinished = false);
    }
}
