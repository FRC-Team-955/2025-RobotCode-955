package frc.robot.autos;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorDashboard.CoralScoringLevel;
import frc.robot.autos.AutoBuilder.IntakeScorePair;
import frc.robot.subsystems.superstructure.ReefAlign.LocalReefSide;
import frc.robot.subsystems.superstructure.ReefAlign.ReefZoneSide;
import frc.robot.subsystems.superstructure.StationAlign.Station;

import java.util.List;

public class BargeSideAuto {
    public static Command get(AutoRoutine routine, boolean alternate) {
        final var firstScoreTraj = routine.trajectory("Barge Side", 0);
        final var secondStationTraj = routine.trajectory("Barge Side", 1);
        final var secondScoreTraj = routine.trajectory("Barge Side", 2);
        final var thirdStationTraj = routine.trajectory("Barge Side", 3);
        final var thirdScoreTraj = routine.trajectory("Barge Side", 4);
        final var fourthStationTraj = routine.trajectory("Barge Side", 5);
        final var fourthScoreTraj = routine.trajectory("Barge Side", 6);
        final var fourthScoreTrajAlternate = routine.trajectory("Barge Side - Fourth Score Alternate", 0);

        return AutoBuilder.createScoring(routine, List.of(
                new IntakeScorePair(null, null, firstScoreTraj, ReefZoneSide.LeftBack, LocalReefSide.Right, CoralScoringLevel.L4),
                new IntakeScorePair(secondStationTraj, Station.BargeSide, secondScoreTraj, ReefZoneSide.LeftFront, LocalReefSide.Left, CoralScoringLevel.L4),
                new IntakeScorePair(thirdStationTraj, Station.BargeSide, thirdScoreTraj, ReefZoneSide.LeftFront, LocalReefSide.Right, CoralScoringLevel.L4),
                alternate ?
                        new IntakeScorePair(fourthStationTraj, Station.BargeSide, fourthScoreTrajAlternate, ReefZoneSide.LeftBack, LocalReefSide.Right, CoralScoringLevel.L2) :
                        new IntakeScorePair(fourthStationTraj, Station.BargeSide, fourthScoreTraj, ReefZoneSide.MiddleFront, LocalReefSide.Left, CoralScoringLevel.L4)
        ));
    }
}
