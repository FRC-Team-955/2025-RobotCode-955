package frc.robot.autos;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorDashboard.LocalReefSide;
import frc.robot.OperatorDashboard.ReefZoneSide;
import frc.robot.autos.AutoBuilder.IntakeScorePair;
import frc.robot.subsystems.elevator.Elevator;

import java.util.List;

public class CenterAuto {
    public static Command get(AutoRoutine routine) {
        final var firstScoreTraj = routine.trajectory("Center");

        return AutoBuilder.createScoring(routine, List.of(
                new IntakeScorePair(null, null, firstScoreTraj, ReefZoneSide.MiddleBack, LocalReefSide.Left, Elevator.Goal.SCORE_L4)
        ));
    }
}
