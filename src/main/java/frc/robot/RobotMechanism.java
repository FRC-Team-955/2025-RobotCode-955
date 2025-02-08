package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Holds the Mechanism2d and all roots and ligaments that visualizes the robot state */
public class RobotMechanism {
    @AutoLogOutput(key = "RobotState/Mechanism")
    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1, 2.1, new Color8Bit(Color.kBlack));

    public final Elevator elevator = new Elevator();
    public final EndEffector endEffector = new EndEffector();
    public final Indexer indexer = new Indexer();
    public final Pivot pivot = new Pivot();

    public class Pivot {
        public final LoggedMechanismRoot2d pivotRoot = mechanism.getRoot("intake", 0.5 - Units.inchesToMeters(10), Units.inchesToMeters(4));
        // TODO: Figure out
        public final LoggedMechanismLigament2d pivotLigament = pivotRoot.append(new LoggedMechanismLigament2d(
                "intake",
                Units.inchesToMeters(20),
                90,
                10,
                new Color8Bit(Color.kOrange)
        ));
    }

    public class Elevator {
        public final LoggedMechanismRoot2d stage1Root = mechanism.getRoot("elevator_stage1", 0, 0);
        public final LoggedMechanismRoot2d stage2Root = mechanism.getRoot("elevator_stage2", 0, 0);
        public final LoggedMechanismRoot2d stage3Root = mechanism.getRoot("elevator_stage3", 0, 0);

        private Elevator() {
            var baseRoot = mechanism
                    .getRoot("elevatorBase", 0.5 + Units.inchesToMeters(7) - 0.06, Units.inchesToMeters(1.85));
            baseRoot.append(new LoggedMechanismLigament2d(
                    "base",
                    Units.inchesToMeters(33.2),
                    90,
                    13,
                    new Color8Bit(new Color(0.2, 0.2, 0.2))
            ));
            stage1Root.append(new LoggedMechanismLigament2d(
                    "stage1",
                    Units.inchesToMeters(32.5),
                    90,
                    12,
                    new Color8Bit(new Color(0.3, 0.3, 0.3))
            ));
            stage2Root.append(new LoggedMechanismLigament2d(
                    "stage2",
                    Units.inchesToMeters(32),
                    90,
                    11,
                    new Color8Bit(new Color(0.4, 0.4, 0.4))
            ));
            stage3Root.append(new LoggedMechanismLigament2d(
                    "stage3",
                    Units.inchesToMeters(7),
                    90,
                    10,
                    new Color8Bit(new Color(0.5, 0.5, 0.5))
            ));
        }
    }

    public class EndEffector {
        public final LoggedMechanismRoot2d root = mechanism.getRoot("endEffector", 0, 0);
        public final LoggedMechanismLigament2d ligament = root.append(new LoggedMechanismLigament2d(
                "ligament",
                Units.inchesToMeters(10),
                90,
                10,
                new Color8Bit(Color.kOrange)
        ));

        public final LoggedMechanismRoot2d beamBreakRoot = mechanism.getRoot("endEffector_beamBreak", 0, 0);
        public final LoggedMechanismLigament2d beamBreakLigament = beamBreakRoot.append(new LoggedMechanismLigament2d(
                "beamBreak",
                Units.inchesToMeters(1),
                0,
                11,
                new Color8Bit(Color.kRed)
        ));

        private EndEffector() {
        }
    }

    public class Indexer {
        private static final double indexerX = 0.5 + Units.inchesToMeters(-12.2);
        private static final double indexerY = Units.inchesToMeters(6);
        private static final double indexerAngle = 13.815;

        public final LoggedMechanismRoot2d root = mechanism.getRoot("indexer", indexerX, indexerY + 0.065);
        public final LoggedMechanismLigament2d ligament = root.append(new LoggedMechanismLigament2d(
                "ligament",
                Units.inchesToMeters(14.75),
                indexerAngle,
                10,
                new Color8Bit(Color.kBlue)
        ));

        public final LoggedMechanismRoot2d beamBreakRoot = mechanism.getRoot("indexer_beamBreak", indexerX + Units.inchesToMeters(9.5), indexerY + Units.inchesToMeters(5));
        public final LoggedMechanismLigament2d beamBreakLigament = beamBreakRoot.append(new LoggedMechanismLigament2d(
                "indexer_beamBreak",
                Units.inchesToMeters(1),
                indexerAngle,
                11,
                new Color8Bit(Color.kRed)
        ));

        private Indexer() {
        }
    }
}
