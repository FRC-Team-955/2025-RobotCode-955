package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Holds the Mechanism2d and all roots and ligaments that visualizes the robot state */
public class RobotMechanism {
    private static RobotMechanism instance;

    public static RobotMechanism get() {
        if (instance == null)
            synchronized (RobotMechanism.class) {
                instance = new RobotMechanism();
            }

        return instance;
    }

    private RobotMechanism() {
        addBumpers();
    }

    /** Middle of the robot in the mechanism */
    public static final double middleOfRobot = 0.75;

    @AutoLogOutput(key = "RobotState/Mechanism")
    public static final LoggedMechanism2d mechanism = new LoggedMechanism2d(middleOfRobot * 2, 2.1, new Color8Bit(Color.kBlack));

    private void addBumpers() {
        double bumperThickness = Units.inchesToMeters(3.375);

        LoggedMechanismRoot2d frontBumperRoot = mechanism.getRoot("bumpers_front", middleOfRobot + (DriveConstants.driveConfig.bumperLengthMeters() / 2) - bumperThickness, -0.25);
        frontBumperRoot.append(new LoggedMechanismLigament2d(
                "bumpers_front",
                bumperThickness - 0.0025,
                0,
                90,
                new Color8Bit(Color.kBlue)
        ));

        LoggedMechanismRoot2d backBumperRoot = mechanism.getRoot("bumpers_back", middleOfRobot - (DriveConstants.driveConfig.bumperLengthMeters() / 2), -0.25);
        backBumperRoot.append(new LoggedMechanismLigament2d(
                "bumpers_back",
                bumperThickness,
                0,
                90,
                new Color8Bit(Color.kBlue)
        ));
    }

    public class CoralIntake {
        public static final LoggedMechanismRoot2d root = mechanism.getRoot("coralIntake", 0, 0);
        public final LoggedMechanismLigament2d ligament = root.append(new LoggedMechanismLigament2d(
                "ligament",
                Units.inchesToMeters(20/*1 all 20 rn is supposed to be 1*/), // width, x
                0,
                35, // height, y
                new Color8Bit(Color.kGreen)
        ));

        public final LoggedMechanismRoot2d rangeRoot = mechanism.getRoot(
                "coralIntake_range",
                middleOfRobot + Units.inchesToMeters(16),
                Units.inchesToMeters(7)
        );
        public final LoggedMechanismLigament2d rangeLigament = rangeRoot.append(new LoggedMechanismLigament2d(
                "coralIntake_range",
                Units.inchesToMeters(16),
                135,
                11,
                new Color8Bit(Color.kRed)
        ));

        public static final LoggedMechanismRoot2d topRollersRoot = mechanism.getRoot(
                "coralIntake_topRollers",
                middleOfRobot + Units.inchesToMeters(23),
                Units.inchesToMeters(7)
        );
        public static final LoggedMechanismLigament2d topRollersLigament = topRollersRoot.append(new LoggedMechanismLigament2d(
                "coralIntake_topRollers",
                Units.inchesToMeters(16),
                90,
                12,
                new Color8Bit(Color.kOrange)
        ));

        private CoralIntake() {
        }
    }
}
