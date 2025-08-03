package frc.lib.motor;

import lombok.Setter;

public class RelativePositionRequestHelper {
    @Setter
    private double positionRad = 0.0;
    private double absoluteSetpointRad = 0.0;
    private double relativeSetpointRad = 0.0;
    private int goalHash = 0;

    public double getAbsoluteSetpointRad(int goalHash, double relativeSetpointRad) {
        // Reset if new goal or new relative setpoint
        if (this.goalHash != goalHash || relativeSetpointRad != this.relativeSetpointRad) {
            this.goalHash = goalHash;
            this.relativeSetpointRad = relativeSetpointRad;

            absoluteSetpointRad = positionRad + this.relativeSetpointRad;

            System.out.println("New relative position setpoint (goal hash = " + this.goalHash + ", relative setpoint = " + this.relativeSetpointRad + ", absolute setpoint = " + absoluteSetpointRad);
        }

        return absoluteSetpointRad;
    }
}
