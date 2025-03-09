package frc.robot.autos;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.OperatorDashboard;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.superstructure.AutoAlignLocations;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.commands.CommandsExt;

import java.util.List;

public class AutoBuilder {
    public static Command createScoring(
            AutoRoutine routine,
            List<IntakeScorePair> trajectories
    ) {
        final Superstructure superstructure = Superstructure.get();

        var ref = new Object() {
            boolean isFinished = false;
        };

        if (trajectories.isEmpty()) {
            return Commands.none();
        }

        IntakeScorePair first = trajectories.stream().findFirst().get();

        routine.active().onTrue(
                Commands.sequence(
                        first.scoreTraj.resetOdometry(),
                        first.scoreTraj.cmd()
                )
        );

        IntakeScorePair last = first;
        // If there's only one trajectory, skip to the finish score
        if (trajectories.size() > 1) {
            for (IntakeScorePair next : trajectories) {
                // Skip first trajectory
                if (next == first) continue;

                last.scoreTraj.atTime("score").onTrue(Commands.sequence(
                        last.scoreCommand(superstructure),
                        CommandsExt.schedule(next.stationTraj.cmd()) // schedule so subsystems run their default commands and so the command doesn't cancel itself
                ));

                next.stationTraj.atTime("intake").onTrue(Commands.sequence(
                        superstructure.funnelIntakeWithAutoAlign(true, next.station),
                        next.scoreTraj.cmd()
                ));

                last = next;
            }
        }

        last.scoreTraj.atTime("score").onTrue(Commands.sequence(
                last.scoreCommand(superstructure),
                Commands.runOnce(() -> ref.isFinished = true)
        ));

        return new WrapperCommand(routine.cmd(() -> ref.isFinished)) {
            @Override
            public void initialize() {
                ref.isFinished = false;
                super.initialize();
            }
        };
    }

    public record IntakeScorePair(
            AutoTrajectory stationTraj,
            AutoAlignLocations.Station station,
            AutoTrajectory scoreTraj,
            OperatorDashboard.ReefZoneSide reefZoneSide,
            OperatorDashboard.LocalReefSide localReefSide
    ) {
        private Command scoreCommand(Superstructure superstructure) {
            return superstructure.autoAlignAndScore(
                    true,
                    () -> reefZoneSide,
                    () -> localReefSide,
                    () -> Elevator.Goal.SCORE_L4,
                    () -> true,
                    () -> false
            );
        }
    }
}
