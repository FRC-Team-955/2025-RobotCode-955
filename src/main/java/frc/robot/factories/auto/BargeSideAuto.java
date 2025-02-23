package frc.robot.factories.auto;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.OperatorDashboard.LocalReefSide;
import frc.robot.OperatorDashboard.ReefZoneSide;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.commands.CommandsExt;

public class BargeSideAuto {
    public static Command get(AutoRoutine routine) {
        final var superstructure = Superstructure.get();

        final var tenOclockLeftTraj = routine.trajectory("Barge Side", 0);
        final var tenOclockRightTraj = routine.trajectory("Barge Side", 1);
        final var eightOclockLeftTraj = routine.trajectory("Barge Side", 2);
        final var eightOclockRightTraj = routine.trajectory("Barge Side", 3);

        var ref = new Object() {
            boolean isFinished = false;
        };

        routine.active().onTrue(
                Commands.sequence(
                        tenOclockLeftTraj.resetOdometry(),
                        tenOclockLeftTraj.cmd()
                )
        );

        tenOclockLeftTraj.atTime("score").onTrue(
                superstructure.autoAlignAndScore(
                        true,
                        () -> ReefZoneSide.LeftBack,
                        () -> LocalReefSide.Left,
                        () -> Elevator.Goal.SCORE_L4,
                        () -> false,
                        () -> false
                ).andThen(CommandsExt.schedule(tenOclockRightTraj.cmd().alongWith(superstructure.funnelIntake(true, false)))) // schedule so subsystems run their default commands and so the command doesn't cancel itself
        );

        tenOclockRightTraj.atTime("score").onTrue(
                superstructure.autoAlignAndScore(
                        true,
                        () -> ReefZoneSide.LeftBack,
                        () -> LocalReefSide.Right,
                        () -> Elevator.Goal.SCORE_L4,
                        () -> false,
                        () -> false
                ).andThen(CommandsExt.schedule(eightOclockLeftTraj.cmd().alongWith(superstructure.funnelIntake(true, false)))) // schedule so subsystems run their default commands and so the command doesn't cancel itself
        );

        eightOclockLeftTraj.atTime("score").onTrue(
                superstructure.autoAlignAndScore(
                        true,
                        () -> ReefZoneSide.LeftFront,
                        () -> LocalReefSide.Left,
                        () -> Elevator.Goal.SCORE_L4,
                        () -> false,
                        () -> false
                ).andThen(CommandsExt.schedule(eightOclockRightTraj.cmd().alongWith(superstructure.funnelIntake(true, false)))) // schedule so subsystems run their default commands and so the command doesn't cancel itself
        );

        eightOclockRightTraj.atTime("score").onTrue(
                superstructure.autoAlignAndScore(
                        true,
                        () -> ReefZoneSide.LeftFront,
                        () -> LocalReefSide.Right,
                        () -> Elevator.Goal.SCORE_L4,
                        () -> false,
                        () -> false
                ).andThen(Commands.runOnce(() -> ref.isFinished = true))
        );

        return routine.cmd(() -> ref.isFinished).beforeStarting(() -> ref.isFinished = false);
    }
}
