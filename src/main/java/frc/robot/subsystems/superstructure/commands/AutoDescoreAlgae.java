package frc.robot.subsystems.superstructure.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.subsystems.superstructure.ReefAlign;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureCommand;
import frc.robot.subsystems.superstructure.SuperstructureContext;
import lombok.RequiredArgsConstructor;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

@RequiredArgsConstructor
public class AutoDescoreAlgae extends SuperstructureCommand {
    private final Supplier<ReefAlign.ReefZoneSide> reefZoneSideSupplier;
    private final BooleanSupplier forceCondition;

    @Override
    public Command create() {
        Command driveTo = Commands.race(
                // Drive to position
                drive.moveTo(() -> ReefAlign.getDescoreAlignPose(reefZoneSideSupplier.get()), false),
                CommandsExt.eagerSequence(
                        Commands.parallel(
                                superstructure.setGoal(Superstructure.Goal.AUTO_DESCORE_ALGAE_WAIT_UNTIL_CAN_RAISE),
                                Commands.waitUntil(() -> ReefAlign.descoreCanRaiseElevator(robotState.getPose(), reefZoneSideSupplier.get()))
                        ),
                        Commands.parallel(
                                superstructure.setGoal(Superstructure.Goal.AUTO_DESCORE_ALGAE_WAIT_FOR_ALIGN),
                                Commands.waitUntil(() -> ReefAlign.descoreIsAligned(robotState.getPose(), reefZoneSideSupplier.get()))
                        )
                )
        );

        Command waitAlgae = Commands.parallel(
                Commands.race(
                        drive.runRobotRelative(() -> new ChassisSpeeds(-0.4, 0, 0)),
                        CommandsExt.eagerSequence(
                                Commands.waitSeconds(0.5),
                                endEffector.waitUntilDescoreAlgaeAmperageTriggered()
                        )
                ),
                superstructure.setGoal(Superstructure.Goal.AUTO_DESCORE_ALGAE_WAIT_FOR_AMPERAGE)
        );

        Timer driveBackTimer = new Timer();
        Command driveBack = drive.runRobotRelative(() -> new ChassisSpeeds(driveBackTimer.get() * 4.0, 0, 0))
                .withTimeout(0.5)
                .deadlineFor(
                        Commands.runOnce(driveBackTimer::restart),
                        superstructure.setGoal(Superstructure.Goal.AUTO_DESCORE_ALGAE_MOVE_BACK)
                );

        Command waitForForce = CommandsExt.eagerSequence(
                Commands.waitSeconds(2),
                Commands.waitUntil(forceCondition)
        );

        return CommandsExt.eagerSequence(
                superstructure.initCtx(SuperstructureContext.reefSideOnly(reefZoneSideSupplier)),
                Commands.race(
                        CommandsExt.eagerSequence(
                                driveTo,
                                waitAlgae
                        ),
                        waitForForce
                ),
                driveBack
        );
    }
}
