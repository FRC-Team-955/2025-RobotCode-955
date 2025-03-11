package frc.robot.autos;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorDashboard.LocalReefSide;
import frc.robot.OperatorDashboard.ReefZoneSide;
import frc.robot.autos.AutoBuilder.IntakeScorePair;
import frc.robot.subsystems.superstructure.AutoAlignLocations.Station;

import java.util.List;

public class ProcessorSideFriendlyAuto {
    public static Command get(AutoRoutine routine) {
        final var firstScoreTraj = routine.trajectory("Processor Side Friendly", 0);
        final var secondStationTraj = routine.trajectory("Processor Side Friendly", 1);
        final var secondScoreTraj = routine.trajectory("Processor Side Friendly", 2);

        return AutoBuilder.createScoring(routine, List.of(
                new IntakeScorePair(null, null, firstScoreTraj, ReefZoneSide.MiddleFront, LocalReefSide.Right),
                new IntakeScorePair(secondStationTraj, Station.ProcessorSideFriendly, secondScoreTraj, ReefZoneSide.MiddleFront, LocalReefSide.Left)
        ));
    }
}
