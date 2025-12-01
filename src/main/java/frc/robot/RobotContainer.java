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
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.goals.WheelRadiusCharacterizationGoal;
import frc.robot.subsystems.gamePieceVision.GamePieceVision;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import frc.robot.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.function.Supplier;

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
    public final IntakePivot intakePivot = IntakePivot.get();
    public final IntakeRoller intakeRoller = IntakeRoller.get();
    public final GamePieceVision gamePieceVision = GamePieceVision.get();
    public final Superstructure superstructure = Superstructure.get();
    public final AprilTagVision aprilTagVision = AprilTagVision.get();
    private final Supplier<OperatorDashboard.CoralScoringLevel> coralScoringLevelSupplier
            =
            operatorDashboard::getSelectedCoralScoringLevel;


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
//        drive.setDefaultCommand(drive.runRobotRelative(() -> {
//            if (Timer.getTimestamp() % 2 >= 1) {
//                return ChassisSpeeds.fromFieldRelativeSpeeds(
//
//                        1, 0,
//                        0, new Rotation2d(0));
//            } else {
//                return ChassisSpeeds.fromFieldRelativeSpeeds(
//                        -1, 0, 0,
//                        new Rotation2d(0));
//            }
//        }));

        drive.setDefaultCommand(drive.driveJoystick());
        superstructure.setDefaultCommand(superstructure.cancel());
//        intakePivot.setDefaultCommand(run(() ->
//                intakePivot.setGoals(IntakePivotGoal.STOW).schedule())
//        );
//        intakeRoller.setDefaultCommand(run(() ->
//                intakeRoller.setGoals(IntakeRollerGoal.IDLE).alongWith
//                        (intakeRoller.setRunning(false))))
//        ;
//        intakePivot.setDefaultCommand(run(() -> {
//
//                    if (Timer.getTimestamp() % 2 >= 1) {
//                        intakePivot.setGoals(IntakePivotGoal.STOW).schedule();
//
//
//                    } else {
//
//                        intakePivot.setGoals(IntakePivotGoal.INTAKE).schedule();
//
//                    }
//
//
//                })
//        );
//        intakeRoller.setDefaultCommand(run(() -> {
//
//                    if (Timer.getTimestamp() % 2 >= 1) {
//                        intakeRoller.setDefaultCommand(run(() -> intakeRoller.setGoals(IntakeRollerGoal.IDLE).schedule()));
//
//
//                    } else {
//                        intakeRoller.setDefaultCommand(run(() ->
//                                intakeRoller.setGoals(IntakeRollerGoal.INTAKE).schedule()));
//
//
//                    }
//
//
//                }
//        ));

//       superstructure.setDefaultCommand(superstructure.cancel());
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
        //controller.
//        controller.leftBumper().onTrue(runOnce(intakeRoller::spawnCoral
//        ));

//        controller.a().whileTrue(
//                superstructure.intake().andThen(Commands.idle())
//        );
        controller.x().onTrue(superstructure.cancel());

        controller.y().onTrue(superstructure.resetPos());
        controller.rightTrigger().onTrue(superstructure.eject().andThen(Commands.idle()));
        //controller.leftBumper().onTrue(superstructure.autoAlign());
        controller.leftBumper().onTrue(superstructure.autoIntakeCoral());
        controller.leftTrigger().onTrue(superstructure.autoScoreCoral(
                operatorDashboard::getSelectedReefZoneSide,
                operatorDashboard::getSelectedLocalReefSide,
                operatorDashboard::getSelectedCoralScoringLevel,
                controller.leftTrigger()));
        controller.a().onTrue(superstructure.score().andThen(Commands.idle()));
//        Trigger canAutoScore = new Trigger(() -> ReefAlign.isAlignable(robotState.getPose(), operatorDashboard.getSelectedReefZoneSide()));
//        controller.leftTrigger()
//                .and(manualScoring.negate())
//                .and(canAutoScore)
//                .onTrue(superstructure.autoScoreCoral(
//                        operatorDashboard::getSelectedReefZoneSide,
//                        operatorDashboard::getSelectedLocalReefSide,
//                        operatorDashboard::getSelectedCoralScoringLevel,
//                        controller.leftTrigger()
//                ));
        //  controller.leftTrigger().onTrue(superstructure.autoAlignToCoral());


//                intakePivot.setGoals(IntakePivotGoal.INTAKE).alongWith(
//                        intakeRoller.setGoals(IntakeRollerGoal.INTAKE))));
//                .())
//                }
//        ;
//
//
//                intakePivot.setGoals(IntakePivotGoal.STOW).alongWith(
//                     );
//


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
