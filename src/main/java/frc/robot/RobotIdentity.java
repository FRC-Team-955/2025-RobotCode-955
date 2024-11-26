package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public enum RobotIdentity {
    COMPBOT(""),
    NEO_DRIVEBASE(""),
    SIMBOT(null);

    public final String macAddress;

    RobotIdentity(String macAddress) {
        this.macAddress = macAddress;
    }

    public static RobotIdentity determine() {
        if (RobotBase.isReal()) {
            final var macAddress = Util.getMacAddress();

            if (macAddress.equals(COMPBOT.macAddress))
                return COMPBOT;

            if (macAddress.equals(NEO_DRIVEBASE.macAddress))
                return NEO_DRIVEBASE;

            System.out.println("Mac address " + macAddress + " did not match any robot identities. Assuming COMPBOT");
            return COMPBOT;
        } else {
            if (Constants.Simulation.shouldReplay)
                return Constants.Simulation.replayIdentity;

            return RobotIdentity.SIMBOT;
        }
    }
}