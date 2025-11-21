package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

public class SuperstructureIOSim extends SuperstructureIO {
    private static final Translation2d[] stationLocations = {
            new Translation2d(1, 1),
            new Translation2d(1, 7),
            new Translation2d(16.5, 7),
            new Translation2d(16.5, 1)
    };

    private final IntakeSimulation intakeSimulation = IntakeSimulation.OverTheBumperIntake(
            // Specify the type of game pieces that the intake can collect
            "Coral",
            // Specify the drivetrain to which this intake is attached
            ModuleIOSim.driveSimulation,
            // Width of the intake
            Meters.of(driveConfig.trackWidthMeters()),
            // The extension length of the intake beyond the robot's frame (when activated)
            Inches.of(10),
            // The intake is mounted on the back side of the chassis
            IntakeSimulation.IntakeSide.BACK,
            // The intake can hold up to 1 note
            1);

    private final RobotState robotState = RobotState.get();
    private final IntakePivot intakePivot = IntakePivot.get();
    private final IntakeRollers intakeRollers = IntakeRollers.get();

    private final Timer sinceCoralIntaked = new Timer();
    private static final double indexTime = 1;
    private final Timer sinceStartedHandoff = new Timer();
    private static final double handoffTime = 0.5;
    private final Timer sinceAtStation = new Timer();
    private static final double stationIntakeTime = 1.5 - indexTime;
    private CoralState coralState = CoralState.SCORING;


    private enum CoralState {
        NO_CORAL,
        INDEXING,
        SCORING
    }

    public SuperstructureIOSim() {
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> coralState = CoralState.SCORING));
    }

    @Override
    public void updateInputs(SuperstructureIOInputs inputs) {
        Logger.recordOutput("Superstructure/CoralState", coralState);
        if (intakeRollers.getIntakeRollersGoal() == IntakeRollers.IntakeRollersGoal.INTAKE && coralState == CoralState.NO_CORAL) {
            intakeSimulation.startIntake();
        } else {
            intakeSimulation.stopIntake();
        }
        var intakedCoral = intakeSimulation.getGamePiecesAmount() > 0;
        if (intakedCoral && coralState == CoralState.NO_CORAL) {
            coralState = CoralState.INDEXING;
            intakeSimulation.obtainGamePieceFromIntake();
            sinceCoralIntaked.restart();
        }

        var pose = robotState.getPose();
        Transform3d coralRobotRelative = null;
        switch (coralState) {
            case NO_CORAL -> {
//                var current = robotState.getPose().getTranslation();
//                if (Arrays.stream(stationLocations).anyMatch(t -> t.getDistance(current) < 1.5) ) {
//                    if (!sinceAtStation.isRunning()) {
//                        sinceAtStation.restart();
//                    }
//                    if (sinceAtStation.hasElapsed(stationIntakeTime)) {
//                        coralState = CoralState.INDEXING;
//                        sinceCoralIntaked.restart();
//                    }
//                } else {
//                    sinceAtStation.stop();
//                }
            }
            case INDEXING -> {
                if (sinceCoralIntaked.hasElapsed(indexTime)) {
                    coralState = CoralState.SCORING;
                }
                var interp = MathUtil.clamp(sinceCoralIntaked.get() / indexTime, 0, 1);
                coralRobotRelative = new Transform3d(
                        Units.inchesToMeters(20) - Units.inchesToMeters(15) * interp,
                        0,
                        Units.inchesToMeters(7) + Units.inchesToMeters(4) * interp,
                        new Rotation3d(
                                0,
                                Units.degreesToRadians(30),
                                MathUtil.clamp(
                                        Units.degreesToRadians(80) - Units.degreesToRadians(90) * interp,
                                        0,
                                        90
                                )
                        )
                );
            }
            case SCORING -> {
                var angle = Units.degreesToRadians(45);
                var coralOffsetX = Units.inchesToMeters(-8.5) + Units.inchesToMeters(6) * Math.tan(angle);
                var coralOffsetZ = Units.inchesToMeters(13.5) +  Units.inchesToMeters(4) * Math.tan(angle);
                coralState = CoralState.NO_CORAL;
                SimulatedArena.getInstance()
                        .addGamePieceProjectile(new ReefscapeCoralOnFly(
                                pose.getTranslation(),
                                new Translation2d(coralOffsetX - Units.inchesToMeters(2), 0),
                                ModuleIOSim.driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                pose.getRotation(),
                                Meters.of(coralOffsetZ + Units.inchesToMeters(2)),
                                MetersPerSecond.of(-1),
                                Degrees.of(65)
                        ));
            }
        }
        switch (coralState) {
            case NO_CORAL-> {
                inputs.intakeRangeMeters = Double.MAX_VALUE;
            }
            case INDEXING, SCORING -> {
                inputs.intakeRangeMeters = 0;
            }
        }
    }

}
