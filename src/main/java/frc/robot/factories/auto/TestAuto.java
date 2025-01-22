package frc.robot.factories.auto;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class TestAuto {
    public static Command get(AutoRoutine routine) {
        final var traj = routine.trajectory("Test Path");

        routine.active().onTrue(
                Commands.sequence(
                        traj.resetOdometry(),
                        traj.cmd()
                )
        );

        // Finish after traj is done being followed
        return routine.cmd(traj.done());
    }
}
