package frc.robot.factories.auto;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

        tenOclockLeftTraj.atTime("raise").onTrue(
                superstructure.scoreCoralDuringAuto(
                        tenOclockLeftTraj.recentlyDone(),
                        () -> Elevator.Goal.SCORE_L4
                ).andThen(CommandsExt.schedule(tenOclockRightTraj.cmd())) // schedule so we don't cancel the current traj
        );

        tenOclockRightTraj.atTime("raise").onTrue(
                superstructure.scoreCoralDuringAuto(
                        tenOclockRightTraj.recentlyDone(),
                        () -> Elevator.Goal.SCORE_L4
                ).andThen(CommandsExt.schedule(eightOclockLeftTraj.cmd())) // schedule so we don't cancel the current traj
        );

        eightOclockLeftTraj.atTime("raise").onTrue(
                superstructure.scoreCoralDuringAuto(
                        eightOclockLeftTraj.recentlyDone(),
                        () -> Elevator.Goal.SCORE_L4
                ).andThen(CommandsExt.schedule(eightOclockRightTraj.cmd())) // schedule so we don't cancel the current traj
        );

        eightOclockRightTraj.atTime("raise").onTrue(
                superstructure.scoreCoralDuringAuto(
                        eightOclockRightTraj.recentlyDone(),
                        () -> Elevator.Goal.SCORE_L4
                ).andThen(Commands.runOnce(() -> ref.isFinished = true))
        );

        return routine.cmd(() -> ref.isFinished).beforeStarting(() -> ref.isFinished = false);
    }
}
