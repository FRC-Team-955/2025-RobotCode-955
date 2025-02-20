package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DigitalInput;

public class SuperstructureIOReal extends SuperstructureIO {
    private final DigitalInput endEffectorBeamBreak = new DigitalInput(0);


    @Override
    public void updateInputs(SuperstructureIOInputs inputs) {
        inputs.endEffectorBeamBreakTriggered = !endEffectorBeamBreak.get();
    }
}
