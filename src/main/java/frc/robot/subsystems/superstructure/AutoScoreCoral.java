package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.OperatorDashboard;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import lombok.RequiredArgsConstructor;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.scoreCoralL1SettleSeconds;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.scoreCoralSettleSeconds;


@RequiredArgsConstructor
public class AutoScoreCoral extends SuperstructureCommands {
    private final Supplier<ReefAlign.ReefZoneSide> reefSideSupplier;
    private final Supplier<ReefAlign.LocalReefSide> sideSupplier;
    private final Supplier<OperatorDashboard.CoralScoringLevel> coralScoringLevelSupplier;
    private final BooleanSupplier forceCondition;

    @Override
    public Command create() {
        Supplier<Pose2d> alignPoseSupplier = () -> ReefAlign.getAlignPose(robotState.getPose(), reefSideSupplier.get(), sideSupplier.get());

        Command initial = Commands.race(
                // Drive to initial position
                drive.moveTo(alignPoseSupplier, false),
                Commands.parallel(
                        superstructure.setGoal(
                                Superstructure.Goal.AUTO_SCORE_CORAL_WAIT_UNTIL_CAN_RAISE,
                                () -> IntakePivot.IntakePivotGoal.STOW,
                                () -> IntakeRoller.IntakeRollerGoal.IDLE,
                                () -> coralScoringLevelSupplier.get()

                        )
                )
        );

        Command waitFinalAndElevator = CommandsExt.eagerSequence(
                Commands.parallel(
                        superstructure.setGoal(
                                Superstructure.Goal.AUTO_SCORE_CORAL_WAIT_FOR_ALIGN,
                                () -> IntakePivot.IntakePivotGoal.STOW,
                                () -> IntakeRoller.IntakeRollerGoal.IDLE,
                                () -> coralScoringLevelSupplier.get()
                        ),
                        Commands.waitUntil(() -> ReefAlign.atFinalAlign(robotState.getPose(), drive.getMeasuredChassisSpeeds(), reefSideSupplier.get(), sideSupplier.get()))
                ),
                Commands.parallel(
                        superstructure.setGoal(
                                Superstructure.Goal.AUTO_SCORE_CORAL_WAIT_FOR_ELEVATOR,
                                () -> IntakePivot.IntakePivotGoal.STOW,
                                () -> IntakeRoller.IntakeRollerGoal.IDLE,
                                () -> coralScoringLevelSupplier.get()
                        )
                )
        );
        // Don't allow forcing for a bit, then check if force is true
        Command waitForForceAndAlign = CommandsExt.eagerSequence(
                Commands.waitSeconds(2).deadlineFor(drive.moveTo(alignPoseSupplier, false)),
                Commands.waitUntil(forceCondition).deadlineFor(
                        rumble(),
                        drive.moveTo(alignPoseSupplier, true)
                        // We only want to
                )
        );

        Command score = CommandsExt.eagerSequence(
                superstructure.setGoal(
                        Superstructure.Goal.AUTO_SCORE_CORAL_WAIT_BEFORE_SCORING,
                        () -> IntakePivot.IntakePivotGoal.STOW,
                        () -> IntakeRoller.IntakeRollerGoal.IDLE,
                        () -> coralScoringLevelSupplier.get()
                ),
                Commands.waitSeconds(0.3),
                superstructure.setGoal(
                        Superstructure.Goal.AUTO_SCORE_CORAL_SCORING,
                        () -> IntakePivot.IntakePivotGoal.STOW,
                        () -> IntakeRoller.IntakeRollerGoal.IDLE,
                        () -> coralScoringLevelSupplier.get()

                )
        );
        // Wait for coral to settle and send the elevator back down
        Command finalize = Commands.either(
                Commands.waitSeconds(scoreCoralL1SettleSeconds),
                Commands.waitSeconds(scoreCoralSettleSeconds),
                () -> coralScoringLevelSupplier.get() == OperatorDashboard.CoralScoringLevel.L1
        );

        return CommandsExt.eagerSequence(
                aprilTagVision.setTagIdFilter(ReefAlign.reefTagIds),
                initial,
                Commands.race(
                        waitFinalAndElevator,
                        waitForForceAndAlign
                ),
                // At this point, the move to goal is already running so no need to set it again
                CommandsExt.eagerSequence(
                        score,
                        finalize
                )
        );
    }
}
