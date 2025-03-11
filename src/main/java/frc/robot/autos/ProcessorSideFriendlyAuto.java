package frc.robot.autos;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorDashboard;
import frc.robot.autos.AutoBuilder.IntakeScorePair;
import frc.robot.subsystems.superstructure.AutoAlignLocations;

import java.util.List;

public class ProcessorSideFriendlyAuto {
    public static Command get(AutoRoutine routine) {
        final var firstScoreTraj = routine.trajectory("Processor Side Friendly", 0);
        final var secondStationTraj = routine.trajectory("Processor Side Friendly", 1);
        final var secondScoreTraj = routine.trajectory("Processor Side Friendly", 2);

        return AutoBuilder.createScoring(routine, List.of(
                new IntakeScorePair(null, null, firstScoreTraj, OperatorDashboard.ReefZoneSide.MiddleFront, OperatorDashboard.LocalReefSide.Right),
                new IntakeScorePair(secondStationTraj, AutoAlignLocations.Station.ProcessorSideFriendly, secondScoreTraj, OperatorDashboard.ReefZoneSide.MiddleFront, OperatorDashboard.LocalReefSide.Left)
        ));
    }
}
