package frc.robot.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class HandoffFactory {
    public static Command get() {
        final var shooter = Shooter.get();
        final var intake = Intake.get();

        return Commands.sequence(
                Commands.either(
                        Commands.none(),
                        Commands.sequence(
                                Commands.race(
                                        intake.hover(),
                                        shooter.handoffWaitForIntake()
                                                .until(shooter::atGoal)
                                ),
                                Commands.race(
                                        intake.handoffReady()
                                                .until(intake::atGoal),
                                        shooter.handoffWaitForIntake()
                                ),
                                Commands.race(
                                        intake.handoffReady(),
                                        shooter.handoffReady()
                                                .until(shooter::atGoal)
                                )
                        ),
                        shooter::hasNoteDebounced
                ),
                Commands.race(
                        Commands.either(
                                Commands.none(),
                                intake.handoffFeed(),
                                shooter::hasNoteDebounced
                        ),
                        shooter.handoffFeed()
                                .until(shooter::hasNoteDebounced)
                )
        ).withName("Handoff");
    }

}
