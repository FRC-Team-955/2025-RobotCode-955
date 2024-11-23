package frc.robot.factories;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class FourPieceWingAutoFactory {
    public static Command get(AutoFactory factory) {
        final var shooter = Shooter.get();
        final var intake = Intake.get();

        final var loop = factory.newLoop("4 Piece Wing");
        final var StoW1w = factory.trajectory("S-W1w", loop);
        final var W1wtoW1 = factory.trajectory("W1w-W1", loop);
        final var W1toW2w = factory.trajectory("W1-W2w", loop);
        final var W2wtoW2 = factory.trajectory("W2w-W2", loop);
        final var W2toW3w = factory.trajectory("W2-W3w", loop);
        final var W3wtoW3 = factory.trajectory("W3w-W3", loop);

        loop.enabled().onTrue(
                AutoInitFactory.get(loop, "4 Piece Wing", StoW1w::getInitialPose)
                        .finallyDo(shooter::shootCalculatedSpinupInBackground)
                        .andThen(StoW1w.cmd())
        );

        final var INTAKE_TIMEOUT = 0.3;

        StoW1w.done().onTrue(Commands.sequence(
                CalculatedShootFactory.get()
                        .finallyDo(shooter::shootCalculatedSpinupInBackground),
                W1wtoW1.cmd()
                        .alongWith(intake.intake())
        ));
        W1wtoW1.done().onTrue(Commands.sequence(
                intake.intake().withTimeout(INTAKE_TIMEOUT),
                Commands.parallel(
                        W1toW2w.cmd(),
                        HandoffFactory.get()
                                .finallyDo(shooter::shootCalculatedSpinupInBackground)
                ),
                CalculatedShootFactory.get()
                        .finallyDo(shooter::shootCalculatedSpinupInBackground),
                W2wtoW2.cmd()
                        .alongWith(intake.intake())
        ));
        W2wtoW2.done().onTrue(Commands.sequence(
                intake.intake().withTimeout(INTAKE_TIMEOUT),
                Commands.parallel(
                        W2toW3w.cmd(),
                        HandoffFactory.get()
                                .finallyDo(shooter::shootCalculatedSpinupInBackground)
                ),
                CalculatedShootFactory.get()
                        .finallyDo(shooter::shootCalculatedSpinupInBackground),
                W3wtoW3.cmd()
                        .alongWith(intake.intake())
        ));
        W3wtoW3.done().onTrue(Commands.sequence(
                intake.intake().withTimeout(INTAKE_TIMEOUT),
                HandoffFactory.get()
                        .finallyDo(shooter::shootCalculatedSpinupInBackground),
                CalculatedShootFactory.get()
        ));

        return loop.cmd();
    }
}
