package frc.robot.autos;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorDashboard.LocalReefSide;
import frc.robot.OperatorDashboard.ReefZoneSide;
import frc.robot.autos.AutoBuilder.IntakeScorePair;
import frc.robot.subsystems.superstructure.AutoAlignLocations.Station;

import java.util.List;

public class ProcessorSideAuto {
    public static Command get(AutoRoutine routine) {
        final var firstScoreTraj = routine.trajectory("Processor Side", 0);
        final var secondStationTraj = routine.trajectory("Processor Side", 1);
        final var secondScoreTraj = routine.trajectory("Processor Side", 2);
        final var thirdStationTraj = routine.trajectory("Processor Side", 3);
        final var thirdScoreTraj = routine.trajectory("Processor Side", 4);

        return AutoBuilder.createScoring(routine, List.of(
                new IntakeScorePair(null, null, firstScoreTraj, ReefZoneSide.RightBack, LocalReefSide.Left),
                new IntakeScorePair(secondStationTraj, Station.ProcessorSide, secondScoreTraj, ReefZoneSide.RightFront, LocalReefSide.Right),
                new IntakeScorePair(thirdStationTraj, Station.ProcessorSide, thirdScoreTraj, ReefZoneSide.RightFront, LocalReefSide.Left)
        ));
    }
}
