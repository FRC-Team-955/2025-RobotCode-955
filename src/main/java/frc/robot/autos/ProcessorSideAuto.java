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
    public static Command get(AutoRoutine routine, Type type) {
        return AutoBuilder.createScoring(routine, List.of(
                new IntakeScorePair(
                        null,
                        null,
                        routine.trajectory("Processor Side", 0),
                        ReefZoneSide.RightBack,
                        LocalReefSide.Left,
                        CoralScoringLevel.L4
                ),
                new IntakeScorePair(
                        routine.trajectory("Processor Side", 1),
                        Station.ProcessorSide,
                        routine.trajectory("Processor Side", 2),
                        ReefZoneSide.RightFront,
                        LocalReefSide.Right,
                        CoralScoringLevel.L4
                ),
                switch (type) {
                    case Normal, AvoidMiddleFront -> new IntakeScorePair(
                            routine.trajectory("Processor Side", 3),
                            Station.ProcessorSide,
                            routine.trajectory("Processor Side", 4),
                            ReefZoneSide.RightFront,
                            LocalReefSide.Left,
                            CoralScoringLevel.L4
                    );
                    case AvoidMiddleFrontAndAdjacent -> new IntakeScorePair(
                            routine.trajectory("Processor Side", 3),
                            Station.ProcessorSide,
                            routine.trajectory("Processor Side - Alternate Right"),
                            ReefZoneSide.RightBack,
                            LocalReefSide.Right,
                            CoralScoringLevel.L4
                    );
                },
                switch (type) {
                    case Normal -> new IntakeScorePair(
                            routine.trajectory("Processor Side", 5),
                            Station.ProcessorSide,
                            routine.trajectory("Processor Side", 6),
                            ReefZoneSide.MiddleFront,
                            LocalReefSide.Right,
                            CoralScoringLevel.L4
                    );
                    case AvoidMiddleFront, AvoidMiddleFrontAndAdjacent -> new IntakeScorePair(
                            routine.trajectory("Processor Side", 5),
                            Station.ProcessorSide,
                            routine.trajectory("Processor Side - Alternate Left"),
                            ReefZoneSide.RightBack,
                            LocalReefSide.Left,
                            CoralScoringLevel.L2
                    );
                }
        ));
    }

    public enum Type {
        Normal,
        AvoidMiddleFront,
        AvoidMiddleFrontAndAdjacent,
    }
}
