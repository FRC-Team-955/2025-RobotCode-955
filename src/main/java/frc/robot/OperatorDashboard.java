package frc.robot;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.subsystem.SubsystemBaseExt;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
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

    private final LoggedNetworkBoolean l1 = new LoggedNetworkBoolean("/Tuning/L1", false);
    private final LoggedNetworkBoolean l2 = new LoggedNetworkBoolean("/Tuning/L2", false);
    private final LoggedNetworkBoolean l3 = new LoggedNetworkBoolean("/Tuning/L3", false);
    private final LoggedNetworkBoolean l4 = new LoggedNetworkBoolean("/Tuning/L4", true);

    @Getter
    private int side = 0;

    @Getter
    private int level = 4;

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

    private void setAllSidesFalse() {
        leftFront.set(false);
        middleFront.set(false);
        rightFront.set(false);
        rightBack.set(false);
        middleBack.set(false);
        leftBack.set(false);
    }

    private void setAllLevelsFalse() {
        l1.set(false);
        l2.set(false);
        l3.set(false);
        l4.set(false);
    }

    @Override
    public void periodicBeforeCommands() {
        if (leftFront.get() && side != 0) {
            setAllSidesFalse();
            leftFront.set(true);
            side = 0;
        } else if (middleFront.get() && side != 1) {
            setAllSidesFalse();
            middleFront.set(true);
            side = 1;
        } else if (rightFront.get() && side != 2) {
            setAllSidesFalse();
            rightFront.set(true);
            side = 2;
        } else if (rightBack.get() && side != 3) {
            setAllSidesFalse();
            rightBack.set(true);
            side = 3;
        } else if (middleBack.get() && side != 4) {
            setAllSidesFalse();
            middleBack.set(true);
            side = 4;
        } else if (leftBack.get() && side != 5) {
            setAllSidesFalse();
            leftBack.set(true);
            side = 5;
        }
        if (l1.get() && level != 1) {
            setAllLevelsFalse();
            l1.set(true);
            level = 1;
        } else if (l2.get() && level != 2) {
            setAllLevelsFalse();
            l2.set(true);
            level = 2;
        } else if (l3.get() && level != 3) {
            setAllLevelsFalse();
            l3.set(true);
            level = 3;
        } else if (l4.get() && level != 4) {
            setAllLevelsFalse();
            l4.set(true);
            level = 4;
        }

        Logger.recordOutput("OperatorDashboard/side", side);
        Logger.recordOutput("OperatorDashboard/level", getElevatorLevel());
    }

    public boolean getLeftSide() {
        return leftSideAlign.get();
    }

    public Elevator.Goal getElevatorLevel() {
        if (level == 1) {
            return Elevator.Goal.SCORE_L1;
        } else if (level == 2) {
            return Elevator.Goal.SCORE_L2;
        } else if (level == 3) {
            return Elevator.Goal.SCORE_L3;
        } else if (level == 4) {
            return Elevator.Goal.SCORE_L4;
        }
        return Elevator.Goal.STOW;
    }
}
