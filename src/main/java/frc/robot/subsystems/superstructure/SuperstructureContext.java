package frc.robot.subsystems.superstructure;

import frc.robot.OperatorDashboard;
import frc.robot.Util;

import java.util.function.Supplier;

public record SuperstructureContext(
        Supplier<OperatorDashboard.CoralScoringLevel> levelSupplier,
        Supplier<ReefAlign.ReefZoneSide> reefSideSupplier
) {
    public static SuperstructureContext levelOnly(Supplier<OperatorDashboard.CoralScoringLevel> levelSupplier) {
        return new SuperstructureContext(
                levelSupplier,
                () -> {
                    Util.error("Reef side supplier used when only level was provided");
                    return ReefAlign.ReefZoneSide.MiddleFront;
                }
        );
    }

    public static SuperstructureContext reefSideOnly(Supplier<ReefAlign.ReefZoneSide> reefSideSupplier) {
        return new SuperstructureContext(
                () -> {
                    Util.error("Level supplier used when only reef side was provided");
                    return OperatorDashboard.CoralScoringLevel.L1;
                },
                reefSideSupplier
        );
    }

    public OperatorDashboard.CoralScoringLevel level() {
        return levelSupplier.get();
    }

    public ReefAlign.ReefZoneSide reefSide() {
        return reefSideSupplier.get();
    }
}
