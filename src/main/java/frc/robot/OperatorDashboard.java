package frc.robot;

import frc.robot.util.subsystem.SubsystemBaseExt;
import lombok.Getter;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class OperatorDashboard extends SubsystemBaseExt {
    /* Dashboard Inputs */
    private final LoggedNetworkBoolean leftSideAlign = new LoggedNetworkBoolean("/Tuning/AlignLeftSide", true);
    private final LoggedNetworkBoolean leftFront = new LoggedNetworkBoolean("/Tuning/leftFront", false);
    private final LoggedNetworkBoolean middleFront = new LoggedNetworkBoolean("/Tuning/middleFront", false);
    private final LoggedNetworkBoolean rightFront = new LoggedNetworkBoolean("/Tuning/rightFront", false);
    private final LoggedNetworkBoolean rightBack = new LoggedNetworkBoolean("/Tuning/rightBack", false);
    private final LoggedNetworkBoolean middleBack = new LoggedNetworkBoolean("/Tuning/middleBack", false);
    private final LoggedNetworkBoolean leftBack = new LoggedNetworkBoolean("/Tuning/leftBack", false);
    @Getter
    private int side = 0;

    private static OperatorDashboard instance;

    public static OperatorDashboard get() {
        if (instance == null)
            synchronized (OperatorDashboard.class) {
                instance = new OperatorDashboard();
            }

        return instance;
    }

    public OperatorDashboard() {
    }

    private void setAllFalse() {
        leftFront.set(false);
        middleFront.set(false);
        rightFront.set(false);
        rightBack.set(false);
        middleBack.set(false);
        leftBack.set(false);
    }

    @Override
    public void periodicBeforeCommands() {
        if (leftFront.get() && side != 0) {
            setAllFalse();
            leftFront.set(true);
            side = 0;
        } else if (middleFront.get() && side != 1) {
            setAllFalse();
            middleFront.set(true);
            side = 1;
        } else if (rightFront.get() && side != 2) {
            setAllFalse();
            rightFront.set(true);
            side = 2;
        } else if (rightBack.get() && side != 3) {
            setAllFalse();
            rightBack.set(true);
            side = 3;
        } else if (middleBack.get() && side != 4) {
            setAllFalse();
            middleBack.set(true);
            side = 4;
        } else if (leftBack.get() && side != 5) {
            setAllFalse();
            leftBack.set(true);
            side = 5;
        }
    }

    public boolean getLeftSide() {
        return leftSideAlign.get();
    }
}
