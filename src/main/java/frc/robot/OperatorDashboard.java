package frc.robot;

import frc.robot.util.network.LoggedNetworkBooleanExt;
import frc.robot.util.subsystem.VirtualSubsystem;
import lombok.Getter;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import java.util.Arrays;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.IntFunction;

public class OperatorDashboard extends VirtualSubsystem {
    public static final LoggedNetworkBooleanExt coastOverride = new LoggedNetworkBooleanExt("/Tuning/OperatorDashboard/CoastOverride", false);
    public static final LoggedNetworkBooleanExt useRealElevatorState = new LoggedNetworkBooleanExt("/Tuning/OperatorDashboard/UseRealElevatorState", false);

    private final Map<ReefZoneSide, LoggedNetworkBoolean> reefZoneSides = generateTogglesForEnum("ReefZoneSides", ReefZoneSide.values());
    private final Map<LocalReefSide, LoggedNetworkBoolean> localReefSides = generateTogglesForEnum("LocalReefSides", LocalReefSide.values());
    private final Map<CoralLevel, LoggedNetworkBoolean> coralLevels = generateTogglesForEnum("CoralLevels", CoralLevel.values());

    @Getter
    private ReefZoneSide selectedReefZoneSide = ReefZoneSide.LeftFront;
    @Getter
    private LocalReefSide selectedLocalReefSide = LocalReefSide.Left;
    @Getter
    private CoralLevel selectedCoralLevel = CoralLevel.L4;

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
        handleEnumToggles(reefZoneSides, selectedReefZoneSide, selectNew -> selectedReefZoneSide = selectNew);
        handleEnumToggles(localReefSides, selectedLocalReefSide, selectNew -> selectedLocalReefSide = selectNew);
        handleEnumToggles(coralLevels, selectedCoralLevel, selectNew -> selectedCoralLevel = selectNew);
    }

    public enum ReefZoneSide {
        LeftFront,
        MiddleFront,
        RightFront,
        RightBack,
        MiddleBack,
        LeftBack
    }

    public enum LocalReefSide {
        Left,
        Right
    }

    public enum CoralLevel {
        L1,
        L2,
        L3,
        L4
    }

    private static <E extends Enum<E>> void handleEnumToggles(
            Map<E, LoggedNetworkBoolean> map,
            E currentlySelected,
            Consumer<E> select
    ) {
        // If none are toggled
        if (map.values().stream().noneMatch(LoggedNetworkBoolean::get)) {
            // Enable the last selected one
            map.get(currentlySelected).set(true);
        } else {
            // Otherwise, look for changes in the toggles
            for (var entry : map.entrySet()) {
                LoggedNetworkBoolean toggle = entry.getValue();
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

    private static <E extends Enum<E>> Map<E, LoggedNetworkBoolean> generateTogglesForEnum(String name, E[] values) {
        return Map.ofEntries(
                Arrays.stream(values)
                        .map(side -> Map.entry(
                                side,
                                new LoggedNetworkBoolean("/Tuning/OperatorDashboard/" + name + "/" + side.name(), false)
                        ))
                        .toArray((IntFunction<Map.Entry<E, LoggedNetworkBoolean>[]>) Map.Entry[]::new)
        );
    }
}
