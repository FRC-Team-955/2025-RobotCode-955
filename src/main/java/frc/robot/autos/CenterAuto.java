package frc.robot.autos;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorDashboard.CoralScoringLevel;
import frc.robot.autos.AutoBuilder.IntakeScorePair;
import frc.robot.subsystems.superstructure.ReefAlign.LocalReefSide;
import frc.robot.subsystems.superstructure.ReefAlign.ReefZoneSide;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.commands.CommandsExt;

import java.util.List;

public class CenterAuto {
    public static Command get(AutoRoutine routine, boolean descore) {
        final var firstScoreTraj = routine.trajectory("Center");

        Command auto = AutoBuilder.createScoring(routine, List.of(
                new IntakeScorePair(null, null, firstScoreTraj, ReefZoneSide.MiddleBack, LocalReefSide.Left, CoralScoringLevel.L4)
        ));

        return descore ? CommandsExt.eagerSequence(
                auto,
                Superstructure.get().autoDescoreAlgae(() -> ReefZoneSide.MiddleBack, () -> true).asProxy()
        ) : auto;
    }
}
