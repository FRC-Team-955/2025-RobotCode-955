package frc.robot.subsystems.superstructure.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.subsystems.superstructure.ReefAlign;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureCommand;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.scoreCoralSettleSeconds;

@RequiredArgsConstructor
public class AutoScoreCoral extends SuperstructureCommand {
    private final Supplier<ReefAlign.ReefZoneSide> reefSideSupplier;
    private final Supplier<ReefAlign.LocalReefSide> sideSupplier;
//    private final Supplier<OperatorDashboard.CoralScoringLevel> coralScoringLevelSupplier;
    private final BooleanSupplier forceCondition;

    @Override
    public Command create() {
        Supplier<Pose2d> alignPoseSupplier = () ->
                ReefAlign.getAlignPose(robotState.getPose(), 0.0, reefSideSupplier.get(), sideSupplier.get());
        Logger.recordOutput("Reef Side Supplier",reefSideSupplier.get());

        Command initial = Commands.race(
                drive.moveTo(alignPoseSupplier, false),
                Commands.parallel(
                        superstructure.setGoal(
                                Superstructure.Goal.AUTO_SCORE_CORAL_WAIT
                        )
                )
        );

        Command waitFinalAlign = CommandsExt.eagerSequence(
                Commands.parallel(
                        superstructure.setGoal(
                                Superstructure.Goal.AUTO_SCORE_CORAL_WAIT
                        ),
                        Commands.waitUntil(() ->
                                ReefAlign.atFinalAlign(robotState.getPose(),
                                        drive.getMeasuredChassisSpeeds(),
                                        reefSideSupplier.get(),
                                        sideSupplier.get())
                        )
                )
        );

        Command waitForForceAndAlign = CommandsExt.eagerSequence(
                Commands.waitSeconds(2).deadlineFor(drive.moveTo(alignPoseSupplier, false)),
                Commands.waitUntil(forceCondition).deadlineFor(
                        rumble(),
                        drive.moveTo(alignPoseSupplier, true)
                )
        );

        Command score = CommandsExt.eagerSequence(
                superstructure.setGoal(
                        Superstructure.Goal.AUTO_SCORE_CORAL_SCORING
                ),
                Commands.waitSeconds(0.3),
                superstructure.setGoal(
                        Superstructure.Goal.AUTO_SCORE_CORAL_SCORING
                )
        );

//        Command finalize = Commands.either(
////                Commands.waitSeconds(scoreCoralL1SettleSeconds),
//                Commands.waitSeconds(scoreCoralSettleSeconds),
//                () -> coralScoringLevelSupplier.get() == OperatorDashboard.CoralScoringLevel.L1
//        );

        return CommandsExt.eagerSequence(
                aprilTagVision.setTagIdFilter(ReefAlign.reefTagIds),
                initial,
                Commands.race(
                        waitFinalAlign,
                        waitForForceAndAlign
                ),
                CommandsExt.eagerSequence(
                        score
                )
        );
    }
}