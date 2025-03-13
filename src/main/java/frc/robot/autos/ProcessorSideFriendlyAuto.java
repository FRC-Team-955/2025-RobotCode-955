package frc.robot.autos;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorDashboard.LocalReefSide;
import frc.robot.OperatorDashboard.ReefZoneSide;
import frc.robot.autos.AutoBuilder.IntakeScorePair;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.superstructure.AutoAlignLocations.Station;

import java.util.List;

public class ProcessorSideFriendlyAuto {
    public static Command get(AutoRoutine routine) {
        final var firstScoreTraj = routine.trajectory("Processor Side Friendly", 0);
        final var secondStationTraj = routine.trajectory("Processor Side Friendly", 1);
        final var secondScoreTraj = routine.trajectory("Processor Side Friendly", 2);
        final var thirdStationTraj = routine.trajectory("Processor Side Friendly", 3);
        final var thirdScoreTraj = routine.trajectory("Processor Side Friendly", 4);

        return AutoBuilder.createScoring(routine, List.of(
                new IntakeScorePair(null, null, firstScoreTraj, ReefZoneSide.MiddleFront, LocalReefSide.Right, Elevator.Goal.SCORE_L4),
                new IntakeScorePair(secondStationTraj, Station.ProcessorSideFriendly, secondScoreTraj, ReefZoneSide.MiddleFront, LocalReefSide.Left, Elevator.Goal.SCORE_L4),
                new IntakeScorePair(thirdStationTraj, Station.ProcessorSideFriendly, thirdScoreTraj, ReefZoneSide.MiddleFront, LocalReefSide.Right, Elevator.Goal.SCORE_L3)
        ));
    }
}
