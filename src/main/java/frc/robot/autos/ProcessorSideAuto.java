package frc.robot.autos;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorDashboard;
import frc.robot.autos.AutoBuilder.IntakeScorePair;
import frc.robot.subsystems.superstructure.AutoAlignLocations;

import java.util.List;

public class ProcessorSideAuto {
    public static Command get(AutoRoutine routine) {
        final var firstScoreTraj = routine.trajectory("Processor Side", 0);
        final var secondStationTraj = routine.trajectory("Processor Side", 1);
        final var secondScoreTraj = routine.trajectory("Processor Side", 2);
        final var thirdStationTraj = routine.trajectory("Processor Side", 3);
        final var thirdScoreTraj = routine.trajectory("Processor Side", 4);

        return AutoBuilder.createScoring(routine, List.of(
                new IntakeScorePair(null, null, firstScoreTraj, OperatorDashboard.ReefZoneSide.RightBack, OperatorDashboard.LocalReefSide.Left),
                new IntakeScorePair(secondStationTraj, AutoAlignLocations.Station.ProcessorSide, secondScoreTraj, OperatorDashboard.ReefZoneSide.RightFront, OperatorDashboard.LocalReefSide.Right),
                new IntakeScorePair(thirdStationTraj, AutoAlignLocations.Station.ProcessorSide, thirdScoreTraj, OperatorDashboard.ReefZoneSide.RightFront, OperatorDashboard.LocalReefSide.Left)
        ));
    }
}
