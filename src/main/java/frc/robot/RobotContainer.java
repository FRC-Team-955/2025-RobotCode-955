package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.CANLogger;
import frc.lib.commands.CommandsExt;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.goals.WheelRadiusCharacterizationGoal;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.intakerollers.IntakeRollersIO;
import frc.robot.subsystems.superstructure.ReefAlign;
import frc.robot.subsystems.superstructure.Superstructure;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.BuildConstants.mode;

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

    /* Subsystems */
    // Note: order does matter
    public final Drive drive = Drive.get();
    public final AprilTagVision aprilTagVision = AprilTagVision.get();
    public final GamePieceVision gamePieceVision = GamePieceVision.get();

    public final IntakePivot intakePivot = IntakePivot.get();
    public final IntakeRollers intakeRollers = IntakeRollers.get();

    public final Superstructure superstructure = Superstructure.get();

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
        drive.setDefaultCommand(Drive.get().driveJoystick());
        superstructure.setDefaultCommand(superstructure.idle().ignoringDisable(true));
//        drive.setDefaultCommand(
//                drive.runRobotRelative(() -> (Timer.getTimestamp() % 2 >= 1 ?
//                        ChassisSpeeds.fromFieldRelativeSpeeds(
//                                1,
//                                0,
//                                0.5,
//                                new Rotation2d(0)
//                        ) :
//                        ChassisSpeeds.fromFieldRelativeSpeeds(
//                                -1,
//                                0,
//                                1,
//                                new Rotation2d(0)
//                        )
//                )
//                )
//        );

//        intakePivot.setDefaultCommand(
//                run(() -> {
//                    double time = Timer.getTimestamp();
//                    if ((time % 2) >= 1) {
//                        intakePivot.setGoals(IntakePivot.IntakePivotGoal.STOW).schedule();
//                    } else {
//                        intakePivot.setGoals(IntakePivot.IntakePivotGoal.INTAKE).schedule();
//                    }
//                })
//        );
//
//        intakeRollers.setDefaultCommand(
//                run(() -> {
//                    double time = Timer.getTimestamp();
//                    if ((time % 2) >= 1) {
//                        intakeRollers.setGoals(IntakeRollers.IntakeRollersGoal.IDLE).schedule();
//                        intakeRollers.setRunning(false);
//                    } else {
//                        intakeRollers.setGoals(IntakeRollers.IntakeRollersGoal.INTAKE).schedule();
//                        intakeRollers.setRunning(true);
//                    }
//                })
//        );
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        controller.y().onTrue(robotState.resetRotation());
        controller.leftBumper().whileTrue(superstructure.cancel());

        if (mode == BuildConstants.Mode.SIM) {
            controller.x().onTrue(Commands.runOnce(() ->
                    SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
                            new Pose2d(Units.inchesToMeters(650), Units.inchesToMeters(30), new Rotation2d(Math.random() * 2 * Math.PI))
                    ))
            ));
            controller.a().onTrue(Commands.runOnce(() ->
                    SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
                            new Pose2d(Units.inchesToMeters(650), Units.inchesToMeters(285), new Rotation2d(Math.random() * 2 * Math.PI))
                    ))
            ));
        }

        Trigger canAutoScore =
                new Trigger(() -> ReefAlign.isAlignable(robotState.getPose(),
                        operatorDashboard.getSelectedReefZoneSide()));

        Trigger hasCoral = new Trigger(superstructure::hasCoral);

        // 🔥 NEW AUTO-ALIGN BUTTON (replaces old Left Trigger + Right Bumper)
        controller.rightBumper().onTrue(
                CommandsExt.eagerSequence(
                        superstructure.cancel(),
                        Commands.either(
                                superstructure.autoScoreCoral(
                                        operatorDashboard::getSelectedReefZoneSide,
                                        operatorDashboard::getSelectedLocalReefSide,
                                        controller.rightBumper()
                                ),

                                Commands.sequence(
                                        Commands.either(
                                                superstructure.moveToCoralReal(),
                                                superstructure.moveToCoralSim(),
                                                () -> mode == BuildConstants.Mode.REAL
                                        ),
                                        superstructure.autoScoreCoral(
                                                operatorDashboard::getSelectedReefZoneSide,
                                                operatorDashboard::getSelectedLocalReefSide,
                                                controller.rightBumper()
                                        )
                                ),

                                superstructure::hasCoral
                        )
                )
        );
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
