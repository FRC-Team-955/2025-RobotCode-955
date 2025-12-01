package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.subsystems.intakeRoller.IntakeRollerIOSim;

public class SuperstructureIOSim extends SuperstructureIO {

    Timer timer = new Timer();
    private CoralState coralState = CoralState.HAS_CORAL;
    private RobotState robotState = RobotState.get();
    private final Timer sinceAtStation = new Timer();
    private final Timer sinceCoralIntaked = new Timer();
    private static final double stationIntakeTime = 0.3;

    private static final Translation2d[] stationLocations = {
            new Translation2d(1, 1),
            new Translation2d(1, 7),
            new Translation2d(16.5, 7),
            new Translation2d(16.5, 1)
    };

    private enum CoralState {
        NO_CORAL,
        INTAKING,
        HAS_CORAL
    }

    public SuperstructureIOSim() {
//        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> coralState = CoralState.HAS_CORAL));
    }

    @Override
    public void updateInputs(SuperstructureIOInputs inputs) {
        {
            if (IntakeRollerIOSim.intakeSimulation.getGamePiecesAmount() != 0) {
                timer.start();
                inputs.intakeCANRangeTriggered = !timer.hasElapsed(0.1);
            } else {
                inputs.intakeCANRangeTriggered = false;
            }
//            var pose = robotState.getPose();
//            switch (coralState) {
//                case NO_CORAL -> {
//                    var current = robotState.getPose().getTranslation();
//
//                    if (
//                            Arrays.stream(stationLocations)
//                                    .anyMatch(t -> t.getDistance(current) < 1.5)
//
//                                    && IntakeRollerIOSim.intakeSimulation.getGamePiecesAmount() == 0
//                    ) {
//                        if (!sinceAtStation.isRunning()) sinceAtStation.restart();
//
//                        if (sinceAtStation.hasElapsed(stationIntakeTime)) {
//                            coralState = CoralState.INTAKING;
//                            sinceCoralIntaked.restart();
//                        }
//                    } else {
//                        sinceAtStation.stop();
//                        sinceAtStation.reset();
//                    }
//                }
//                case INTAKING -> {
//
//                }
//                case HAS_CORAL -> {
//
//                }

        }

    }
}


