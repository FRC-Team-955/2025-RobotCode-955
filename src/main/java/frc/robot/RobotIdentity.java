package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

public enum RobotIdentity {
    COMPBOT("00-80-2F-22-AE-61"),
    ALPHABOT("00-80-2F-38-8E-B8"),
    SIMBOT(null);

    public final String macAddress;

    RobotIdentity(String macAddress) {
        this.macAddress = macAddress;
    }

    public static RobotIdentity determine() {
        if (RobotBase.isReal()) {
            final var macAddress = Util.getMacAddress();

            Logger.recordMetadata("MacAddress", macAddress);

            if (macAddress.equals(COMPBOT.macAddress))
                return COMPBOT;

            if (macAddress.equals(ALPHABOT.macAddress))
                return ALPHABOT;

            System.out.println("Mac address " + macAddress + " did not match any robot identities. Assuming COMPBOT");
            return COMPBOT;
        } else {
            if (Constants.Simulation.shouldReplay)
                return Constants.Simulation.replayIdentity;

            return RobotIdentity.SIMBOT;
        }
    }
}