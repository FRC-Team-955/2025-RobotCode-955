package frc.robot.autos;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorDashboard;
import frc.robot.autos.AutoBuilder.IntakeScorePair;
import frc.robot.subsystems.superstructure.AutoAlignLocations;

import java.util.List;

public class BargeSideAuto {
    public static Command get(AutoRoutine routine) {
        final var firstScoreTraj = routine.trajectory("Barge Side", 0);
        final var secondStationTraj = routine.trajectory("Barge Side", 1);
        final var secondScoreTraj = routine.trajectory("Barge Side", 2);
        final var thirdStationTraj = routine.trajectory("Barge Side", 3);
        final var thirdScoreTraj = routine.trajectory("Barge Side", 4);

        return AutoBuilder.createScoring(routine, List.of(
                new IntakeScorePair(null, null, firstScoreTraj, OperatorDashboard.ReefZoneSide.LeftBack, OperatorDashboard.LocalReefSide.Right),
                new IntakeScorePair(secondStationTraj, AutoAlignLocations.Station.BargeSide, secondScoreTraj, OperatorDashboard.ReefZoneSide.LeftFront, OperatorDashboard.LocalReefSide.Left),
                new IntakeScorePair(thirdStationTraj, AutoAlignLocations.Station.BargeSide, thirdScoreTraj, OperatorDashboard.ReefZoneSide.LeftFront, OperatorDashboard.LocalReefSide.Right)
        ));
    }
}
