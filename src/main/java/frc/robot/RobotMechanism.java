package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intake.Intake;
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

    public final IntakeMechanism PivotMechanism;

    private RobotMechanism() {
        addBumpers();
        PivotMechanism = new IntakeMechanism();
    }

    /** Middle of the robot in the mechanism */
    public static final double middleOfRobot = 0.75;

    @AutoLogOutput(key = "RobotState/Mechanism")
    public final LoggedMechanism2d mechanism = new LoggedMechanism2d(middleOfRobot * 2, 2.1, new Color8Bit(Color.kBlack));

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

    public class IntakeMechanism {
        private static final double x = middleOfRobot - Units.inchesToMeters(12);
        private static final double y = Units.inchesToMeters(6);

        public final LoggedMechanismRoot2d root = mechanism.getRoot("intake_root", x, y);

        private static final double upperLength = Units.inchesToMeters(10);
        public final LoggedMechanismLigament2d upperArm = root.append(new LoggedMechanismLigament2d(
                "intake_upperArm",
                upperLength,
                90,
                12,
                new Color8Bit(Color.kOrange)
        ));
        private static final double lowerLength = Units.inchesToMeters(8);
        public final LoggedMechanismLigament2d lowerArm = upperArm.append(new LoggedMechanismLigament2d(
                "intake_lowerArm",
                lowerLength,
                0,
                10,
                new Color8Bit(Color.kRed)
        ));
        private static final double rollerLength = Units.inchesToMeters(2);
        public final LoggedMechanismLigament2d roller = lowerArm.append(new LoggedMechanismLigament2d(
                "intake_roller",
                rollerLength,
                0,
                8,
                new Color8Bit(Color.kGray)
        ));

        public IntakeMechanism() {}

        public void update() {
            Intake intake = Intake.get();
            double angleRad = intake.getCurrentAngleRad();
            Intake.IntakeGoal goal = intake.getCurrentGoal();

            double displayAngleDeg = switch (goal) {
                case STOW -> 90;
                case INTAKE -> 180;
                default -> 90;
            };
            upperArm.setAngle(displayAngleDeg);
            lowerArm.setAngle(45);
            roller.setAngle(0);
        }
    }
}
