package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DigitalInput;

public class SuperstructureIOReal extends SuperstructureIO {
    private final DigitalInput endEffectorBeamBreak = new DigitalInput(4);
    private final DigitalInput funnelBeamBreak = new DigitalInput(8);

    public SuperstructureIOReal() {
    }

    @Override
    public void updateInputs(SuperstructureIOInputs inputs) {
        inputs.endEffectorBeamBreakTriggered = !endEffectorBeamBreak.get();
        inputs.funnelBeamBreakTriggered = !funnelBeamBreak.get();
    }
}
