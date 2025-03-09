package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.funnel.Funnel;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

import static edu.wpi.first.units.Units.*;

public class SuperstructureIOSim extends SuperstructureIO {
    private static final Translation2d[] stationLocations = {
            new Translation2d(1, 1),
            new Translation2d(1, 7),
            new Translation2d(16.5, 7),
            new Translation2d(16.5, 1)
    };

    private final RobotState robotState = RobotState.get();
    private final EndEffector endEffector = EndEffector.get();
    private final Elevator elevator = Elevator.get();
    private final Funnel funnel = Funnel.get();

    private final Timer sinceCoralIntaked = new Timer();
    private static final double indexTime = 1;
    private final Timer sinceAtStation = new Timer();
    private static final double stationIntakeTime = 1.5 - indexTime;
    private CoralState coralState = CoralState.IN_END_EFFECTOR; // preload

    private enum CoralState {
        NO_CORAL,
        INTAKING,
        IN_END_EFFECTOR
    }

    public SuperstructureIOSim() {
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> coralState = CoralState.IN_END_EFFECTOR));
    }

    @Override
    public void updateInputs(SuperstructureIOInputs inputs) {
        var pose = robotState.getPose();
        Transform3d coralRobotRelative = null;
        switch (coralState) {
            case NO_CORAL -> {
                var current = robotState.getPose().getTranslation();
                if (Arrays.stream(stationLocations).anyMatch(t -> t.getDistance(current) < 1.5) && endEffector.getRollersGoal() == EndEffector.RollersGoal.FUNNEL_INTAKE) {
                    if (!sinceAtStation.isRunning()) sinceAtStation.restart();
                    if (sinceAtStation.hasElapsed(stationIntakeTime)) {
                        coralState = CoralState.INTAKING;
                        sinceCoralIntaked.restart();
                    }
                } else {
                    sinceAtStation.stop();
                }
            }
            case INTAKING -> {
                if (sinceCoralIntaked.hasElapsed(indexTime) && funnel.getGoal() == Funnel.Goal.INTAKE) {
                    coralState = CoralState.IN_END_EFFECTOR;
                }
                var interp = MathUtil.clamp(sinceCoralIntaked.get() / indexTime, 0, 1);
                coralRobotRelative = new Transform3d(
                        Units.inchesToMeters(5) - Units.inchesToMeters(9) * interp,
                        0,
                        Units.inchesToMeters(11) + Units.inchesToMeters(1) * interp,
                        new Rotation3d(0, Units.degreesToRadians(7), 0)
                );
            }
            case IN_END_EFFECTOR -> {
                var angle = Units.degreesToRadians(-endEffector.getAngleDegrees() - 90);
                var coralOffsetX = Units.inchesToMeters(-8.5) + Units.inchesToMeters(6) * Math.tan(angle);
                // TODO: fix the trig, it doesn't actually work but is good enough for sim
                var coralOffsetZ = Units.inchesToMeters(13.5) + elevator.getPositionMeters() + Units.inchesToMeters(4) * Math.tan(angle);
                if (endEffector.getRollersGoal() == EndEffector.RollersGoal.SCORE_CORAL || endEffector.getRollersGoal() == EndEffector.RollersGoal.EJECT) {
                    coralState = CoralState.NO_CORAL;
                    SimulatedArena.getInstance()
                            .addGamePieceProjectile(new ReefscapeCoralOnFly(
                                    pose.getTranslation(),
                                    new Translation2d(coralOffsetX - Units.inchesToMeters(2), 0),
                                    ModuleIOSim.driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                    pose.getRotation(),
                                    // The height at which the coral is ejected
                                    Meters.of(coralOffsetZ + Units.inchesToMeters(2)),
                                    // The initial speed of the coral
                                    MetersPerSecond.of(-1),
                                    Degrees.of(65)
                            ));
                } else {
                    coralRobotRelative = new Transform3d(
                            coralOffsetX,
                            0,
                            coralOffsetZ,
                            new Rotation3d(0, angle, 0)
                    );
                }
            }
        }
        if (coralRobotRelative != null) {
            Logger.recordOutput("FieldSimulation/CoralInRobot", new Pose3d[]{
                    new Pose3d(
                            pose.getX(),
                            pose.getY(),
                            0,
                            new Rotation3d(0, 0, pose.getRotation().getRadians())
                    ).transformBy(coralRobotRelative)
            });
        } else {
            Logger.recordOutput("FieldSimulation/CoralInRobot", new Pose3d[]{});
        }

        switch (coralState) {
            case INTAKING -> {
                inputs.funnelBeamBreakTriggered = true;
                inputs.endEffectorBeamBreakTriggered = false;
            }
            case IN_END_EFFECTOR -> {
                inputs.funnelBeamBreakTriggered = false;
                inputs.endEffectorBeamBreakTriggered = true;
            }
            case NO_CORAL -> {
                inputs.funnelBeamBreakTriggered = false;
                inputs.endEffectorBeamBreakTriggered = false;
            }
        }
    }
}
