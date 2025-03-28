package frc.robot.autos;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.OperatorDashboard;
import frc.robot.subsystems.superstructure.ReefAlign;
import frc.robot.subsystems.superstructure.StationAlign;
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

        IntakeScorePair first = trajectories.get(0);

        Command startCmd = CommandsExt.eagerSequence(
                first.scoreTraj.resetOdometry(),
                first.scoreTraj.cmd()
        );

        IntakeScorePair last = first;
        // If there's only one trajectory, skip to the finish score
        if (trajectories.size() > 1) {
            for (IntakeScorePair next : trajectories) {
                // Skip first trajectory
                if (next == first || next.station == null || next.stationTraj == null) continue;

                last.scoreTraj.atTime("score").onTrue(CommandsExt.eagerSequence(
                        last.scoreCommand(superstructure),
                        // scheduling the trajectory wastes a cycle; instead, reset the superstructure and run the trajectory at the same time
                        Commands.parallel(
                                superstructure.ensureNotBusyAndResetGoals(),
                                next.stationTraj.cmd()
                        )
                ));

                next.stationTraj.atTime("intake").onTrue(CommandsExt.eagerSequence(
                        superstructure.autoFunnelIntake(true, next.station),
                        // scheduling the trajectory wastes a cycle; instead, reset the superstructure and run the trajectory at the same time
                        Commands.parallel(
                                superstructure.ensureNotBusyAndResetGoals(),
                                next.scoreTraj.cmd()
                        )
                ));

                last = next;
            }
        }

        last.scoreTraj.atTime("score").onTrue(CommandsExt.eagerSequence(
                last.scoreCommand(superstructure),
                Commands.runOnce(() -> ref.isFinished = true)
        ));

        return new WrapperCommand(
                routine.cmd(() -> ref.isFinished)
                        // routine.active() wastes a cycle. We can just start it now as a parallel command
                        .alongWith(startCmd.asProxy())
        ) {
            @Override
            public void initialize() {
                ref.isFinished = false;
                super.initialize();
            }
        };
    }

    public record IntakeScorePair(
            AutoTrajectory stationTraj,
            StationAlign.Station station,
            AutoTrajectory scoreTraj,
            ReefAlign.ReefZoneSide reefZoneSide,
            ReefAlign.LocalReefSide localReefSide,
            OperatorDashboard.CoralScoringLevel coralScoringLevel
    ) {
        private Command scoreCommand(Superstructure superstructure) {
            return superstructure.autoScoreCoral(
                    true,
                    () -> reefZoneSide,
                    () -> localReefSide,
                    () -> coralScoringLevel,
                    () -> true
            );
        }
    }
}
