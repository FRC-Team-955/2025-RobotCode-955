package frc.robot.subsystems.superstructure.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.subsystems.superstructure.StationAlign;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureCommand;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class AutoFunnelIntake extends SuperstructureCommand {
    private final StationAlign.Station station;

    @Override
    public Command create() {
        Command intake = Commands.race(
                waitUntilEndEffectorTriggered(),
                waitUntilFunnelTriggered(),
                gamePieceVision.waitForGamePiece()
        ).deadlineFor(
                CommandsExt.eagerSequence(
                        Commands.parallel(
                                superstructure.setGoal(Superstructure.Goal.AUTO_FUNNEL_INTAKE_WAITING_ALIGN),
                                drive.moveTo(station::getAlignPose, false)
                                        .until(() -> StationAlign.atAlignPose(robotState.getPose(), station))
                        ),
                        Commands.parallel(
                                superstructure.setGoal(Superstructure.Goal.AUTO_FUNNEL_INTAKE_WAITING_SHAKE),
                                shake()
                        )
                )
        );
        return CommandsExt.eagerSequence(
                intake,
                waitUntilEndEffectorTriggered().deadlineFor(superstructure.setGoal(Superstructure.Goal.HANDOFF)),
                superstructure.home()
        );
    }
}
