package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.HighFrequencySamplingThread;

import java.util.Queue;

public class SuperstructureIOReal extends SuperstructureIO {
    private final DigitalInput endEffectorBeamBreak = new DigitalInput(4);
    private final DigitalInput funnelBeamBreak = new DigitalInput(5);

    private final Queue<Double> timestampQueue;
    private final Queue<Boolean> endEffectorBeamBreakTriggeredQueue;
    private final Queue<Boolean> funnelBeamBreakTriggeredQueue;

    public SuperstructureIOReal() {
        timestampQueue = HighFrequencySamplingThread.get().makeTimestampQueue();
        endEffectorBeamBreakTriggeredQueue = HighFrequencySamplingThread.get().registerGenericSignal(endEffectorBeamBreak::get);
        funnelBeamBreakTriggeredQueue = HighFrequencySamplingThread.get().registerGenericSignal(funnelBeamBreak::get);
    }

    @Override
    public void updateInputs(SuperstructureIOInputs inputs) {
        inputs.endEffectorBeamBreakTriggered = !endEffectorBeamBreak.get();
        inputs.funnelBeamBreakTriggered = !funnelBeamBreak.get();

        inputs.highFrequencyTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();

        inputs.highFrequencyEndEffectorBeamBreakTriggered = new boolean[endEffectorBeamBreakTriggeredQueue.size()];
        for (int i = 0; i < endEffectorBeamBreakTriggeredQueue.size(); i++) {
            inputs.highFrequencyEndEffectorBeamBreakTriggered[i] = !endEffectorBeamBreakTriggeredQueue.poll();
        }

        inputs.highFrequencyFunnelBeamBreakTriggered = new boolean[funnelBeamBreakTriggeredQueue.size()];
        for (int i = 0; i < funnelBeamBreakTriggeredQueue.size(); i++) {
            inputs.highFrequencyFunnelBeamBreakTriggered[i] = !funnelBeamBreakTriggeredQueue.poll();
        }

        timestampQueue.clear();
        endEffectorBeamBreakTriggeredQueue.clear();
        funnelBeamBreakTriggeredQueue.clear();
    }
}
