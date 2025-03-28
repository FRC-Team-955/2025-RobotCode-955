package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.JoystickDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.superstructure.ReefAlign;
import frc.robot.subsystems.superstructure.ReefAlign.LocalReefSide;
import frc.robot.subsystems.superstructure.ReefAlign.ReefZoneSide;
import frc.robot.util.network.LoggedNetworkBooleanExt;
import frc.robot.util.network.LoggedNetworkNumberExt;
import frc.robot.util.subsystem.VirtualSubsystem;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

import java.util.Arrays;
import java.util.EnumMap;
import java.util.function.Consumer;

public class OperatorDashboard extends VirtualSubsystem {
    private final RobotState robotState = RobotState.get();
    private final JoystickDrive joystickDrive = JoystickDrive.get();

    private static final String prefix = "/OperatorDashboard/";

    public final LoggedNetworkBooleanExt coastOverride = new LoggedNetworkBooleanExt(prefix + "CoastOverride", false);
    public final LoggedNetworkBooleanExt coralStuckInRobotMode = new LoggedNetworkBooleanExt(prefix + "CoralStuckInRobotMode", false);
    public final LoggedNetworkBooleanExt manualScoring = new LoggedNetworkBooleanExt(prefix + "ManualScoring", false);
    public final LoggedNetworkBooleanExt ignoreEndEffectorBeamBreak = new LoggedNetworkBooleanExt(prefix + "IgnoreEndEffectorBeamBreak", false);
    public final LoggedNetworkBooleanExt disableInterpolateAutoAlign = new LoggedNetworkBooleanExt(prefix + "DisableInterpolateAutoAlign", false);
    public final LoggedNetworkBooleanExt autoChosen = new LoggedNetworkBooleanExt(prefix + "AutoChosen", false);
    public final LoggedNetworkBooleanExt manualReefSide = new LoggedNetworkBooleanExt(prefix + "ManualReefSide", false);
    public final LoggedNetworkBooleanExt forceGamePieceLEDs = new LoggedNetworkBooleanExt(prefix + "ForceGamePieceLEDs", false);

    public final LoggedNetworkBooleanExt elevatorEStop = new LoggedNetworkBooleanExt(prefix + "ElevatorEStop", false);
    public final LoggedNetworkBooleanExt useRealElevatorState = new LoggedNetworkBooleanExt(prefix + "UseRealElevatorState", false);
    public final LoggedNetworkBooleanExt forceZeroElevator = new LoggedNetworkBooleanExt(prefix + "ForceZeroElevator", false);
    public final LoggedNetworkNumberExt elevatorOffsetMeters = new LoggedNetworkNumberExt(prefix + "ElevatorOffsetMeters", 0);
    public final LoggedNetworkBooleanExt trustElevatorFollower = new LoggedNetworkBooleanExt(prefix + "TrustElevatorFollower", false);

    private final EnumMap<ReefZoneSide, LoggedNetworkBooleanExt> reefZoneSides = generateTogglesForEnum("ReefZoneSides", ReefZoneSide.values(), ReefZoneSide.class);
    private final EnumMap<LocalReefSide, LoggedNetworkBooleanExt> localReefSides = generateTogglesForEnum("LocalReefSides", Arrays.stream(LocalReefSide.values()).filter(side -> side != LocalReefSide.Middle).toArray(LocalReefSide[]::new), LocalReefSide.class);
    private final EnumMap<CoralScoringLevel, LoggedNetworkBooleanExt> coralScoringLevels = generateTogglesForEnum("CoralScoringLevels", CoralScoringLevel.values(), CoralScoringLevel.class);

    @Getter
    private ReefZoneSide selectedReefZoneSide = ReefZoneSide.LeftFront;
    @Getter
    private LocalReefSide selectedLocalReefSide = LocalReefSide.Left;
    @Getter
    private CoralScoringLevel selectedCoralScoringLevel = CoralScoringLevel.L4;

    private final Alert coastOverrideAlert = new Alert("Coast override is enabled.", Alert.AlertType.kWarning);
    private final Alert coralStuckInRobotModeAlert = new Alert("Coral stuck in robot mode is enabled.", Alert.AlertType.kWarning);
    private final Alert manualScoringAlert = new Alert("Manual scoring is enabled.", Alert.AlertType.kWarning);
    private final Alert ignoreEndEffectorBeamBreakAlert = new Alert("Ignore end effector beam break is enabled.", Alert.AlertType.kWarning);
    private final Alert autoNotChosenAlert = new Alert("Auto is not chosen!", Alert.AlertType.kError);
    @SuppressWarnings("FieldCanBeLocal")
    private final Alert constantSetAlert = new Alert("Constants are set.", Alert.AlertType.kInfo);
    private final Alert manualReefSideAlert = new Alert("Manual reef side choosing is enabled.", Alert.AlertType.kWarning);

    private final OperatorKeypad operatorKeypad = new OperatorKeypad();
    private final Alert operatorKeypadDisconnectedAlert = new Alert("Operator keypad is not connected!", Alert.AlertType.kError);

    @Setter
    private boolean ignoreClosestReefSideChanges = false;

    private static OperatorDashboard instance;

    public static OperatorDashboard get() {
        if (instance == null)
            synchronized (OperatorDashboard.class) {
                instance = new OperatorDashboard();
            }

        return instance;
    }

    private OperatorDashboard() {
        if (Constants.tuningMode || DriveConstants.disableDriving || DriveConstants.disableGyro) {
            constantSetAlert.set(true);
        }
    }

    @Override
    public void periodicBeforeCommands() {
        // Note - we only handle alerts for general overrides.
        // So subsystem toggles are handled in their respective subsystems
        coastOverrideAlert.set(coastOverride.get());
        coralStuckInRobotModeAlert.set(coralStuckInRobotMode.get());
        manualScoringAlert.set(manualScoring.get());
        ignoreEndEffectorBeamBreakAlert.set(ignoreEndEffectorBeamBreak.get());
        autoNotChosenAlert.set(!autoChosen.get());

        if (operatorKeypad.isConnected()) {
            operatorKeypadDisconnectedAlert.set(false);

            if (operatorKeypad.getManualReefSide()) {
                manualReefSide.set(true);
                ReefZoneSide newReefZoneSide = operatorKeypad.getReefZoneSide();
                if (newReefZoneSide != null) selectedReefZoneSide = newReefZoneSide;
            } else {
                manualReefSide.set(false);
                if (!ignoreClosestReefSideChanges) {
                    selectedReefZoneSide = ReefAlign.determineClosestReefSide(robotState.getPose(), joystickDrive.getSetpointFieldRelative());
                }
            }
            updateToggles(reefZoneSides, selectedReefZoneSide);

            CoralScoringLevel newCoralScoringLevel = operatorKeypad.getCoralScoringLevel();
            if (newCoralScoringLevel != null) selectedCoralScoringLevel = newCoralScoringLevel;
            updateToggles(coralScoringLevels, selectedCoralScoringLevel);

            LocalReefSide newLocalReefSide = operatorKeypad.getLocalReefSide();
            if (newLocalReefSide != null) selectedLocalReefSide = newLocalReefSide;
            updateToggles(localReefSides, selectedLocalReefSide);
        } else {
            operatorKeypadDisconnectedAlert.set(true);

            if (manualReefSide.get()) {
                handleEnumToggles(reefZoneSides, selectedReefZoneSide, selectNew -> selectedReefZoneSide = selectNew);
            } else {
                if (!ignoreClosestReefSideChanges) {
                    selectedReefZoneSide = ReefAlign.determineClosestReefSide(robotState.getPose(), joystickDrive.getSetpointFieldRelative());
                }
                updateToggles(reefZoneSides, selectedReefZoneSide);
            }
            handleEnumToggles(localReefSides, selectedLocalReefSide, selectNew -> selectedLocalReefSide = selectNew);
            handleEnumToggles(coralScoringLevels, selectedCoralScoringLevel, selectNew -> selectedCoralScoringLevel = selectNew);
        }

        manualReefSideAlert.set(manualReefSide.get());
    }

    @RequiredArgsConstructor
    public enum CoralScoringLevel {
        L1(Elevator.Goal.SCORE_L1),
        L2(Elevator.Goal.SCORE_L2),
        L3(Elevator.Goal.SCORE_L3),
        L4(Elevator.Goal.SCORE_L4),
        ;

        public final Elevator.Goal coralScoringElevatorGoal;
    }

    private static <E extends Enum<E>> void updateToggles(
            EnumMap<E, LoggedNetworkBooleanExt> map,
            E currentlySelected
    ) {
        LoggedNetworkBooleanExt toggle = map.get(currentlySelected);
        // Set the corresponding toggle to true
        toggle.set(true);
        // Set the rest to false
        for (var entry1 : map.entrySet()) {
            if (entry1.getKey() != currentlySelected) {
                entry1.getValue().set(false);
            }
        }
    }

    private static <E extends Enum<E>> void handleEnumToggles(
            EnumMap<E, LoggedNetworkBooleanExt> map,
            E currentlySelected,
            Consumer<E> select
    ) {
        // If none are toggled
        if (map.values().stream().noneMatch(LoggedNetworkBooleanExt::get)) {
            // Enable the last selected one
            map.get(currentlySelected).set(true);
        } else {
            // Otherwise, look for changes in the toggles
            for (var entry : map.entrySet()) {
                LoggedNetworkBooleanExt toggle = entry.getValue();
                // If it's toggled
                if (toggle.get()) {
                    E key = entry.getKey();
                    // If it wasn't already selected
                    if (key != currentlySelected) {
                        // Select the new value
                        select.accept(key);
                        // Set the rest to false
                        for (var entry1 : map.entrySet()) {
                            if (entry1.getKey() != key) {
                                entry1.getValue().set(false);
                            }
                        }
                        break;
                    }
                }
            }
        }
    }

    private static <E extends Enum<E>> EnumMap<E, LoggedNetworkBooleanExt> generateTogglesForEnum(String name, E[] enumValues, Class<E> enumClass) {
        return Util.createEnumMap(enumClass, enumValues, (side) -> new LoggedNetworkBooleanExt(prefix + name + "/" + side.name(), false));
    }

    private static class OperatorKeypad {
        private final GenericHID hid = new GenericHID(1);

        public boolean isConnected() {
            return hid.isConnected();
        }

        public ReefZoneSide getReefZoneSide() {
            if (hid.getRawButton(1)) return ReefZoneSide.LeftFront;
            if (hid.getRawButton(2)) return ReefZoneSide.MiddleFront;
            if (hid.getRawButton(3)) return ReefZoneSide.RightFront;
            if (hid.getRawButton(4)) return ReefZoneSide.RightBack;
            if (hid.getRawButton(5)) return ReefZoneSide.MiddleBack;
            if (hid.getRawButton(6)) return ReefZoneSide.LeftBack;
            return null;
        }

        public CoralScoringLevel getCoralScoringLevel() {
            if (hid.getRawButton(7)) return CoralScoringLevel.L1;
            if (hid.getRawButton(8)) return CoralScoringLevel.L2;
            if (hid.getRawButton(9)) return CoralScoringLevel.L3;
            if (hid.getRawButton(10)) return CoralScoringLevel.L4;
            return null;
        }

        public LocalReefSide getLocalReefSide() {
            if (hid.getRawButton(11)) return LocalReefSide.Left;
            if (hid.getRawButton(12)) return LocalReefSide.Right;
            return null;
        }

        public boolean getManualReefSide() {
            if (hid.getRawButton(13)) return true;
            if (hid.getRawButton(14)) return false;
            return true;
        }
    }
}
