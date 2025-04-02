package frc.robot.autos;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorDashboard.CoralScoringLevel;
import frc.robot.autos.AutoBuilder.IntakeScorePair;
import frc.robot.subsystems.superstructure.ReefAlign.LocalReefSide;
import frc.robot.subsystems.superstructure.ReefAlign.ReefZoneSide;
import frc.robot.subsystems.superstructure.StationAlign.Station;

import java.util.List;

public class ProcessorSideAuto {
    public static Command get(AutoRoutine routine, boolean alternate) {
        final var firstScoreTraj = routine.trajectory("Processor Side", 0);
        final var secondStationTraj = routine.trajectory("Processor Side", 1);
        final var secondScoreTraj = routine.trajectory("Processor Side", 2);
        final var thirdStationTraj = routine.trajectory("Processor Side", 3);
        final var thirdScoreTraj = routine.trajectory("Processor Side", 4);
        final var fourthStationTraj = routine.trajectory("Processor Side", 5);
        final var fourthScoreTraj = routine.trajectory("Processor Side", 6);
        final var fourthScoreTrajAlternate = routine.trajectory("Processor Side - Fourth Score Alternate", 0);

        return AutoBuilder.createScoring(routine, List.of(
                new IntakeScorePair(null, null, firstScoreTraj, ReefZoneSide.RightBack, LocalReefSide.Left, CoralScoringLevel.L4),
                new IntakeScorePair(secondStationTraj, Station.ProcessorSide, secondScoreTraj, ReefZoneSide.RightFront, LocalReefSide.Right, CoralScoringLevel.L4),
                new IntakeScorePair(thirdStationTraj, Station.ProcessorSide, thirdScoreTraj, ReefZoneSide.RightFront, LocalReefSide.Left, CoralScoringLevel.L4),
                alternate ?
                        new IntakeScorePair(fourthStationTraj, Station.ProcessorSide, fourthScoreTrajAlternate, ReefZoneSide.RightBack, LocalReefSide.Left, CoralScoringLevel.L2) :
                        new IntakeScorePair(fourthStationTraj, Station.ProcessorSide, fourthScoreTraj, ReefZoneSide.MiddleFront, LocalReefSide.Right, CoralScoringLevel.L4)
        ));
    }
}
