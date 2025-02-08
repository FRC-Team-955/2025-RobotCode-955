package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.factories.auto.TestAuto;
import frc.robot.subsystems.coralintake.CoralIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.CommandNintendoSwitchProController;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Controller
    private final CommandXboxController driverController = Constants.Simulation.useNintendoSwitchProController ? new CommandNintendoSwitchProController(0) : new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    private final LoggedDashboardChooser<Command> characterizationChooser = new LoggedDashboardChooser<>("Characterization Choices");

    private final RobotState robotState = RobotState.get();

    /* Subsystems */
    // Note: order does matter
    private final Drive drive = Drive.get();
    private final Vision vision = Vision.get();
    private final CoralIntake coralIntake = CoralIntake.get();
    private final Indexer indexer = Indexer.get();
    private final Elevator elevator = Elevator.get();
    private final EndEffector endEffector = EndEffector.get();
    private final Superstructure superstructure = Superstructure.get();
    private final LEDs leds = LEDs.get();

    public RobotContainer() {
        robotState.afterSubsystemsInitialized();
        addAutos();
        addCharacterizations();
        setDefaultCommands();
        configureButtonBindings();
    }

    private void addAutos() {
        final var factory = drive.createAutoFactory();

        autoChooser.addDefaultOption("Test Auto", TestAuto.get(factory.newRoutine("Test Auto")));

        autoChooser.addOption("Characterization", Commands.deferredProxy(characterizationChooser::get));
    }

    private void addCharacterizations() {
        ////////////////////// DRIVE //////////////////////

        characterizationChooser.addOption("Drive Wheel Radius Characterization", drive.wheelRadiusCharacterization(Drive.WheelRadiusCharacterization.Direction.CLOCKWISE));
        characterizationChooser.addOption("Drive SysId (Quasistatic Forward)", drive.sysId.quasistatic(SysIdRoutine.Direction.kForward));
        characterizationChooser.addOption("Drive SysId (Quasistatic Reverse)", drive.sysId.quasistatic(SysIdRoutine.Direction.kReverse));
        characterizationChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysId.dynamic(SysIdRoutine.Direction.kForward));
        characterizationChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysId.dynamic(SysIdRoutine.Direction.kReverse));

        ////////////////////// DRIVE //////////////////////

        characterizationChooser.addOption("Elevator SysId (Quasistatic Forward)", elevator.sysId.quasistatic(SysIdRoutine.Direction.kForward));
        characterizationChooser.addOption("Elevator SysId (Quasistatic Reverse)", elevator.sysId.quasistatic(SysIdRoutine.Direction.kReverse));
        characterizationChooser.addOption("Elevator SysId (Dynamic Forward)", elevator.sysId.dynamic(SysIdRoutine.Direction.kForward));
        characterizationChooser.addOption("Elevator SysId (Dynamic Reverse)", elevator.sysId.dynamic(SysIdRoutine.Direction.kReverse));
//        characterizationChooser.addOption("Elevator Gravity", elevator.gravityCharacterization());
    }

    private void setDefaultCommands() {
        drive.setDefaultCommand(
                drive.driveJoystick(
                        // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
                        // forward on joystick is negative y - we want positive x for forward
                        () -> -driverController.getLeftY(),
                        // right on joystick is positive x - we want negative y for right
                        () -> -driverController.getLeftX(),
                        // right on joystick is positive x - we want negative x for right (CCW is positive)
                        () -> -driverController.getRightX(),
                        () -> {
                            var gamepiece = vision.getClosestGamepiece();
                            return gamepiece.map(gamepieceTranslation -> {
                                var relativeToRobot = gamepieceTranslation.minus(robotState.getTranslation());
                                if (relativeToRobot.getNorm() < Units.feetToMeters(1)) {
                                    // Don't try to face towards it if we are too close
                                    return new Pose2d(gamepieceTranslation, robotState.getRotation());
                                } else {
                                    // Try to face towards the game piece
                                    var toGamepiece = new Rotation2d(relativeToRobot.getX(), relativeToRobot.getY());
                                    return new Pose2d(gamepieceTranslation, toGamepiece);
                                }
                            });
                        }
                )
        );

        superstructure.setDefaultCommand(superstructure.idle());
        coralIntake.setDefaultCommand(superstructure.coralIntakeIdle());
        indexer.setDefaultCommand(superstructure.indexerIdle());
        elevator.setDefaultCommand(superstructure.elevatorIdle());
        endEffector.setDefaultCommand(superstructure.endEffectorIdle());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        driverController.y().onTrue(robotState.resetRotation());

        driverController.rightTrigger().whileTrue(superstructure.intakeCoral());

        driverController.leftTrigger().onTrue(superstructure.scoreCoralManual(
                driverController.leftTrigger(),
                driverController.leftBumper()
        ));

//        // Lock to 0° when A button is held
//        controller
//                .a()
//                .whileTrue(
//                        DriveCommands.joystickDriveAtAngle(
//                                drive,
//                                () -> -controller.getLeftY(),
//                                () -> -controller.getLeftX(),
//                                () -> new Rotation2d()));

        // Switch to X pattern when X button is pressed
//        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
