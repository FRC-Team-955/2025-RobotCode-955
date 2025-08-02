package frc.robot.subsystems.superstructure.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureCommand;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class FunnelIntake extends SuperstructureCommand {
    @Override
    public Command create() {
        return CommandsExt.eagerSequence(
                Commands.race(
                        waitUntilEndEffectorTriggered(),
                        waitUntilFunnelTriggered()
                ).deadlineFor(superstructure.setGoal(Superstructure.Goal.FUNNEL_INTAKE_WAITING)),
                waitUntilEndEffectorTriggered().deadlineFor(superstructure.setGoal(Superstructure.Goal.HANDOFF)),
                superstructure.home()
        );
    }
}
