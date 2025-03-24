package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.superstructure.SuperstructureIOSim;

public class GamePieceVisionIOSim extends GamePieceVisionIO {
    private boolean ledsOn = false;

    private final Timer ledsNotOnTimer = new Timer();

    public GamePieceVisionIOSim() {
    }

    @Override
    public void updateInputs(GamePieceVisionIOInputs inputs) {
        inputs.connected = true;
        inputs.ledsOn = ledsOn;
        if (SuperstructureIOSim.gamePieceVisible && !ledsOn) {
            if (!ledsNotOnTimer.isRunning()) {
                ledsNotOnTimer.restart();
            }
            inputs.visible = !ledsNotOnTimer.hasElapsed(0.25);
        } else {
            if (ledsNotOnTimer.isRunning()) {
                ledsNotOnTimer.stop();
            }
            inputs.visible = SuperstructureIOSim.gamePieceVisible;
        }
    }

    @Override
    public void setLEDs(boolean on) {
        ledsOn = on;
    }
}
