package frc.robot.subsystems.superstructure.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.subsystems.superstructure.ReefAlign;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureCommand;
import frc.robot.subsystems.superstructure.SuperstructureContext;
import lombok.RequiredArgsConstructor;

import java.util.function.Supplier;

@RequiredArgsConstructor
public class DescoreAlgaeManual extends SuperstructureCommand {
    private final Supplier<ReefAlign.ReefZoneSide> reefZoneSideSupplier;

    @Override
    public Command create() {
        return CommandsExt.eagerSequence(
                superstructure.initCtx(SuperstructureContext.reefSideOnly(reefZoneSideSupplier)),
                superstructure.setGoal(Superstructure.Goal.DESCORE_ALGAE_WAIT_FOR_ELEVATOR),
                elevator.waitUntilAtGoal(),
                superstructure.setGoal(Superstructure.Goal.DESCORE_ALGAE_DESCORING),
                Commands.idle()
        );
    }
}
