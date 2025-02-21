package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.network.LoggedNetworkBooleanExt;
import frc.robot.util.network.LoggedNetworkNumberExt;
import frc.robot.util.subsystem.VirtualSubsystem;
import lombok.Getter;

import java.util.Arrays;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.IntFunction;

public class OperatorDashboard extends VirtualSubsystem {
    private static final String prefix = "/OperatorDashboard/";

    public final LoggedNetworkBooleanExt coastOverride = new LoggedNetworkBooleanExt(prefix + "CoastOverride", false);
    public final LoggedNetworkBooleanExt coralStuckInRobotMode = new LoggedNetworkBooleanExt(prefix + "CoralStuckInRobotMode", false);
    public final LoggedNetworkBooleanExt manualScoring = new LoggedNetworkBooleanExt(prefix + "ManualScoring", false);

    public final LoggedNetworkBooleanExt elevatorEStop = new LoggedNetworkBooleanExt(prefix + "ElevatorEStop", false);
    public final LoggedNetworkBooleanExt useRealElevatorState = new LoggedNetworkBooleanExt(prefix + "UseRealElevatorState", false);
    public final LoggedNetworkBooleanExt forceZeroElevator = new LoggedNetworkBooleanExt(prefix + "ForceZeroElevator", false);
    public final LoggedNetworkNumberExt elevatorOffsetMeters = new LoggedNetworkNumberExt(prefix + "ElevatorOffsetMeters", 0);

    private final Map<ReefZoneSide, LoggedNetworkBooleanExt> reefZoneSides = generateTogglesForEnum("ReefZoneSides", ReefZoneSide.values());
    private final Map<LocalReefSide, LoggedNetworkBooleanExt> localReefSides = generateTogglesForEnum("LocalReefSides", LocalReefSide.values());
    private final Map<CoralScoringLevel, LoggedNetworkBooleanExt> coralScoringLevels = generateTogglesForEnum("CoralScoringLevels", CoralScoringLevel.values());
    private final Map<AlgaeDescoringLevel, LoggedNetworkBooleanExt> algaeDescoringLevels = generateTogglesForEnum("AlgaeDescoringLevels", AlgaeDescoringLevel.values());

    @Getter
    private ReefZoneSide selectedReefZoneSide = ReefZoneSide.LeftFront;
    @Getter
    private LocalReefSide selectedLocalReefSide = LocalReefSide.Left;
    @Getter
    private CoralScoringLevel selectedCoralScoringLevel = CoralScoringLevel.L4;
    @Getter
    private AlgaeDescoringLevel selectedAlgaeDescoringLevel = AlgaeDescoringLevel.L3;

    private final Alert coastOverrideAlert = new Alert("Coast override is enabled.", Alert.AlertType.kWarning);

    private static OperatorDashboard instance;

    public static OperatorDashboard get() {
        if (instance == null)
            synchronized (OperatorDashboard.class) {
                instance = new OperatorDashboard();
            }

        return instance;
    }

    private OperatorDashboard() {
    }

    @Override
    public void periodicBeforeCommands() {
        coastOverrideAlert.set(coastOverride.get());

        handleEnumToggles(reefZoneSides, selectedReefZoneSide, selectNew -> selectedReefZoneSide = selectNew);
        handleEnumToggles(localReefSides, selectedLocalReefSide, selectNew -> selectedLocalReefSide = selectNew);
        handleEnumToggles(coralScoringLevels, selectedCoralScoringLevel, selectNew -> selectedCoralScoringLevel = selectNew);
        handleEnumToggles(algaeDescoringLevels, selectedAlgaeDescoringLevel, selectNew -> selectedAlgaeDescoringLevel = selectNew);
    }

    public Elevator.Goal getCoralScoringElevatorGoal() {
        return switch (selectedCoralScoringLevel) {
            case L1 -> Elevator.Goal.SCORE_L1;
            case L2 -> Elevator.Goal.SCORE_L2;
            case L3 -> Elevator.Goal.SCORE_L3;
            case L4 -> Elevator.Goal.SCORE_L4;
        };
    }

    public Elevator.Goal getAlgaeDescoringElevatorGoal() {
        return switch (selectedAlgaeDescoringLevel) {
            case L2 -> Elevator.Goal.DESCORE_L2;
            case L3 -> Elevator.Goal.DESCORE_L3;
        };
    }

    public enum ReefZoneSide {
        LeftFront,
        MiddleFront,
        RightFront,
        RightBack,
        MiddleBack,
        LeftBack,
    }

    public enum LocalReefSide {
        Left,
        Right,
    }

    public enum CoralScoringLevel {
        L1,
        L2,
        L3,
        L4,
    }

    public enum AlgaeDescoringLevel {
        L2,
        L3,
    }

    private static <E extends Enum<E>> void handleEnumToggles(
            Map<E, LoggedNetworkBooleanExt> map,
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
                    E side = entry.getKey();
                    // If it wasn't already selected
                    if (side != currentlySelected) {
                        // Select the new value
                        select.accept(side);
                        // Set the rest to false
                        for (var entry1 : map.entrySet()) {
                            if (entry1.getKey() != side) {
                                entry1.getValue().set(false);
                            }
                        }
                        break;
                    }
                }
            }
        }
    }

    private static <E extends Enum<E>> Map<E, LoggedNetworkBooleanExt> generateTogglesForEnum(String name, E[] values) {
        return Map.ofEntries(
                Arrays.stream(values)
                        .map(side -> Map.entry(
                                side,
                                new LoggedNetworkBooleanExt(prefix + name + "/" + side.name(), false)
                        ))
                        .toArray((IntFunction<Map.Entry<E, LoggedNetworkBooleanExt>[]>) Map.Entry[]::new)
        );
    }
}
