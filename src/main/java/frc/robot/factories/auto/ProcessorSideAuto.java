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

        final var twoOclockLeftTraj = routine.trajectory("Processor Side", 0);
        final var twoOclockRightTraj = routine.trajectory("Processor Side", 1);
        final var fourOclockLeftTraj = routine.trajectory("Processor Side", 2);
        final var fourOclockRightTraj = routine.trajectory("Processor Side", 3);

        var ref = new Object() {
            boolean isFinished = false;
        };

        routine.active().onTrue(
                Commands.sequence(
                        twoOclockLeftTraj.resetOdometry(),
                        twoOclockLeftTraj.cmd()
                )
        );

        twoOclockLeftTraj.atTime("score").onTrue(
                superstructure.autoAlignAndScore(
                        true,
                        () -> ReefZoneSide.RightBack,
                        () -> LocalReefSide.Right,
                        () -> Elevator.Goal.SCORE_L4,
                        () -> false
                ).andThen(CommandsExt.schedule(twoOclockRightTraj.cmd().alongWith(superstructure.funnelIntake(true)))) // schedule so subsystems run their default commands and so the command doesn't cancel itself
        );

        twoOclockRightTraj.atTime("score").onTrue(
                superstructure.autoAlignAndScore(
                        true,
                        () -> ReefZoneSide.RightBack,
                        () -> LocalReefSide.Left,
                        () -> Elevator.Goal.SCORE_L4,
                        () -> false
                ).andThen(CommandsExt.schedule(fourOclockLeftTraj.cmd().alongWith(superstructure.funnelIntake(true)))) // schedule so subsystems run their default commands and so the command doesn't cancel itself
        );

        fourOclockLeftTraj.atTime("score").onTrue(
                superstructure.autoAlignAndScore(
                        true,
                        () -> ReefZoneSide.RightFront,
                        () -> LocalReefSide.Right,
                        () -> Elevator.Goal.SCORE_L4,
                        () -> false
                ).andThen(CommandsExt.schedule(fourOclockRightTraj.cmd().alongWith(superstructure.funnelIntake(true)))) // schedule so subsystems run their default commands and so the command doesn't cancel itself
        );

        fourOclockRightTraj.atTime("score").onTrue(
                superstructure.autoAlignAndScore(
                        true,
                        () -> ReefZoneSide.RightFront,
                        () -> LocalReefSide.Left,
                        () -> Elevator.Goal.SCORE_L4,
                        () -> false
                ).andThen(Commands.runOnce(() -> ref.isFinished = true))
        );

        return routine.cmd(() -> ref.isFinished).beforeStarting(() -> ref.isFinished = false);
    }
}
