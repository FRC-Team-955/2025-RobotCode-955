package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.CANLogger;
import frc.lib.commands.CommandsExt;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.goals.WheelRadiusCharacterizationGoal;
import frc.robot.subsystems.drive.intake.CoralIntake;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    private final LoggedDashboardChooser<Command> characterizationChooser = new LoggedDashboardChooser<>("Characterization Choices");

    public final RobotState robotState = RobotState.get();
    public final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    public final Controller controller = Controller.get();
    public final CANLogger canLogger = CANLogger.get();
    public final CoralIntake pivot = CoralIntake.get();
    /* Subsystems */
    // Note: order does matter
    public final Drive drive = Drive.get();

    public RobotContainer() {
        addAutos();
        addCharacterizations();
        setDefaultCommands();
        configureButtonBindings();

        new Trigger(() -> DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() < 30)
                .onTrue(controller.rumble(0.5, 2.0));
    }

    private void addAutos() {
        autoChooser.addOption("None", Commands.none());
        autoChooser.addOption("Leave", drive.runRobotRelative(() -> new ChassisSpeeds(-0.5, 0, 0)).withTimeout(5));

        autoChooser.addOption(
                "Characterization",
                // We need to require the superstructure during characterization so that the default command doesn't get run
                Commands.deferredProxy(() -> CommandsExt.eagerSequence(
                        characterizationChooser.get()
                ))
        );
    }

    private void addCharacterizations() {
        ////////////////////// DRIVE //////////////////////

        // TODO
        characterizationChooser.addOption("Drive 1 m/s Characterization", drive.runRobotRelative(() -> new ChassisSpeeds(1.0, 0.0, 0.0)));
        characterizationChooser.addOption("Drive 2 m/s Characterization", drive.runRobotRelative(() -> new ChassisSpeeds(2.0, 0.0, 0.0)));
        characterizationChooser.addOption("Drive 3 m/s Characterization", drive.runRobotRelative(() -> new ChassisSpeeds(3.0, 0.0, 0.0)));
        characterizationChooser.addOption("Drive 4 m/s Characterization", drive.runRobotRelative(() -> new ChassisSpeeds(4.0, 0.0, 0.0)));
        characterizationChooser.addOption("Drive Full Speed Characterization", drive.fullSpeedCharacterization());
        characterizationChooser.addOption("Drive Wheel Radius Characterization", drive.wheelRadiusCharacterization(WheelRadiusCharacterizationGoal.Direction.CLOCKWISE));
    }

    private void setDefaultCommands() {
        drive.setDefaultCommand(drive.driveJoystick());


        /*drive.setDefaultCommand(
                drive.runRobotRelative(() -> (Timer.getTimestamp() % 2 >= 1 ?
                                ChassisSpeeds.fromFieldRelativeSpeeds(
                                        1,
                                        0,
                                        0.5,
                                        new Rotation2d(0)
                                ) :
                                ChassisSpeeds.fromFieldRelativeSpeeds(
                                        -1,
                                        0,
                                        1,
                                        new Rotation2d(0)
                                )
                        )
                )
        );*/

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // NOTE: if you are binding a trigger to a command returned by a subsystem, you must wrap it in CommandsExt.eagerSequence(superstructure.cancel(), <your command>)
        // You must do this because if you don't, superstructure's default command will cancel your command

        controller.y().onTrue(robotState.resetRotation());

        // NOTE: if you are binding a trigger to a command returned by a subsystem, you must wrap it in CommandsExt.eagerSequence(superstructure.cancel(), <your command>)
        // You must do this because if you don't, superstructure's default command will cancel your command
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
