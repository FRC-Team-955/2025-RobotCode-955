package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.commands.CommandsExt;
import frc.robot.autos.BargeSideAuto;
import frc.robot.autos.CenterAuto;
import frc.robot.autos.ProcessorSideAuto;
import frc.robot.autos.ProcessorSideFriendlyAuto;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.CANLogger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.Optional;

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
    public final Elevator elevator = Elevator.get();
    public final EndEffector endEffector = EndEffector.get();
    public final Funnel funnel = Funnel.get();
    public final Drive drive = Drive.get();
    public final AprilTagVision aprilTagVision = AprilTagVision.get();
    public final GamePieceVision gamePieceVision = GamePieceVision.get();
    public final Superstructure superstructure = Superstructure.get();

    public RobotContainer() {
        addAutos();
        addCharacterizations();
        setDefaultCommands();
        configureButtonBindings();

        new Trigger(() -> DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() < 30)
                .onTrue(controller.rumble(0.5, 2.0));

        new Trigger(superstructure::isForceable)
                .onTrue(controller.rumble(0.5, 0.5));
    }

    private void addAutos() {
        final var factory = drive.createAutoFactory();

        autoChooser.addOption("None", Commands.none());
        autoChooser.addOption("Leave", drive.runRobotRelative(() -> new ChassisSpeeds(-0.5, 0, 0)).withTimeout(5));

        autoChooser.addOption("Barge Side - Normal", BargeSideAuto.get(factory.newRoutine("Barge Side - Normal"), BargeSideAuto.Type.Normal));
        autoChooser.addOption("Barge Side - Avoid Middle Front", BargeSideAuto.get(factory.newRoutine("Barge Side - Avoid Middle Front"), BargeSideAuto.Type.AvoidMiddleFront));
        autoChooser.addOption("Barge Side - Avoid Middle Front And Adjacent", BargeSideAuto.get(factory.newRoutine("Barge Side - Avoid Middle Front And Adjacent"), BargeSideAuto.Type.AvoidMiddleFrontAndAdjacent));

        autoChooser.addOption("Processor Side - Normal", ProcessorSideAuto.get(factory.newRoutine("Processor Side - Normal"), ProcessorSideAuto.Type.Normal));
        autoChooser.addOption("Processor Side - Avoid Middle Front", ProcessorSideAuto.get(factory.newRoutine("Processor Side - Avoid Middle Front"), ProcessorSideAuto.Type.AvoidMiddleFront));
        autoChooser.addOption("Processor Side - Avoid Middle Front And Adjacent", ProcessorSideAuto.get(factory.newRoutine("Processor Side - Avoid Middle Front And Adjacent"), ProcessorSideAuto.Type.AvoidMiddleFrontAndAdjacent));

        autoChooser.addOption("Processor Side - Friendly", ProcessorSideFriendlyAuto.get(factory.newRoutine("Processor Side - Friendly")));
        autoChooser.addOption("Center", CenterAuto.get(factory.newRoutine("Center"), CenterAuto.Type.Normal));
        autoChooser.addOption("Center - Descore", CenterAuto.get(factory.newRoutine("Center - Descore"), CenterAuto.Type.Descore));

        autoChooser.addOption(
                "Characterization",
                // We need to require the superstructure during characterization so that the default command doesn't get run
                Commands.deferredProxy(() -> CommandsExt.eagerSequence(
                        superstructure.cancel(),
                        characterizationChooser.get()
                ))
        );
    }

    private void addCharacterizations() {
        ////////////////////// DRIVE //////////////////////

        characterizationChooser.addOption("Drive Feedforward Characterization", drive.feedforwardCharacterization());
        characterizationChooser.addOption("Drive Full Speed Characterization", drive.fullSpeedCharacterization());
        characterizationChooser.addOption("Drive Wheel Radius Characterization", drive.wheelRadiusCharacterization(Drive.WheelRadiusCharacterization.Direction.CLOCKWISE));
        characterizationChooser.addOption("Drive SysId (Quasistatic Forward)", drive.sysId.quasistatic(SysIdRoutine.Direction.kForward));
        characterizationChooser.addOption("Drive SysId (Quasistatic Reverse)", drive.sysId.quasistatic(SysIdRoutine.Direction.kReverse));
        characterizationChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysId.dynamic(SysIdRoutine.Direction.kForward));
        characterizationChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysId.dynamic(SysIdRoutine.Direction.kReverse));
    }

    private void setDefaultCommands() {
        drive.setDefaultCommand(drive.driveJoystick(Optional::empty));

        superstructure.setDefaultCommand(CommandsExt.eagerSequence(superstructure.cancel(), Commands.idle()).ignoringDisable(true));
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

        driverController.y().onTrue(robotState.resetRotation());

        driverController.leftBumper().onTrue(superstructure.cancel());

        driverController.x().whileTrue(superstructure.eject());

        driverController.a().onTrue(superstructure.home());

        // TODO precondition !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get()
        driverController.rightTrigger().whileTrue(superstructure.funnelIntake().asProxy().repeatedly());

        var ref = new Object() {
            boolean shouldDescoreAlgae = false;
        };
        driverController.leftTrigger().onTrue(Commands.either(
                superstructure.scoreCoralManual( // TODO precondition: endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get()
                        driverController.leftTrigger(),
                        operatorDashboard::getSelectedCoralScoringLevel
                ).asProxy(),
                CommandsExt.eagerSequence(
                        superstructure.autoScoreCoral( // TODO precondtion Only run if you have coral and are in front of your reef side () -> (endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get()) && ReefAlign.isAlignable(robotState.getPose(), reefSideSupplier.get()),
                                operatorDashboard::getSelectedReefZoneSide,
                                operatorDashboard::getSelectedLocalReefSide,
                                operatorDashboard::getSelectedCoralScoringLevel,
                                driverController.leftTrigger(),
                                false
                        ).deadlineFor(
                                Commands.startRun(
                                        () -> ref.shouldDescoreAlgae = false,
                                        () -> {
                                            if (driverController.rightBumper().getAsBoolean()) {
                                                ref.shouldDescoreAlgae = true;
                                            }
                                        }
                                ).until(() -> ref.shouldDescoreAlgae)
                        ),
                        CommandsExt.onlyIf(
                                () -> ref.shouldDescoreAlgae,
                                superstructure.autoDescoreAlgae( // TODO precondition (!endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get()) && ReefAlign.isAlignable(robotState.getPose(), reefSideSupplier.get())
                                        operatorDashboard::getSelectedReefZoneSide,
                                        driverController.rightBumper()
                                )
                        )
                ).asProxy(),
                // Use manual scoring if override enabled or when scoring L1
                () -> operatorDashboard.manualScoring.get()
                        || operatorDashboard.getSelectedCoralScoringLevel() == OperatorDashboard.CoralScoringLevel.L1
        ));

        driverController.rightBumper().onTrue(CommandsExt.onlyIf(
                () -> superstructure.getGoal() == Superstructure.Goal.IDLE,
                Commands.either(
                        superstructure.descoreAlgaeManual( // TODO precondition !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get()
                                operatorDashboard::getSelectedReefZoneSide
                        ).asProxy(),
                        superstructure.autoDescoreAlgae( // TODO precondition (same as auto descore after score) (!endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get()) && ReefAlign.isAlignable(robotState.getPose(), reefSideSupplier.get())
                                operatorDashboard::getSelectedReefZoneSide,
                                driverController.rightBumper()
                        ).asProxy(),
                        operatorDashboard.manualScoring::get
                )
        ));

        // TODO manual elevator see elevator and superstructure and stuff
//        operatorDashboard.operatorKeypad.getOverride4()
//                .or(operatorDashboard.zeroElevator::get)
//                .and(() -> !operatorDashboard.manualElevator.get())
//                .toggleOnTrue(Commands.parallel(
//                        superstructure.zeroElevator(),
//                        // Turn off the toggle instantly so it's like a button
//                        Commands.runOnce(() -> operatorDashboard.zeroElevator.set(false))
//                ).ignoringDisable(true));
//        operatorDashboard.operatorKeypad.getOverride6()
//                .and(() -> !operatorDashboard.manualElevator.get())
//                .onTrue(Commands.runOnce(() -> operatorDashboard.useRealElevatorState.set(true)));
//
//        operatorDashboard.operatorKeypad.getOverride4()
//                .or(operatorDashboard.manualElevatorUp::get)
//                .and(operatorDashboard.manualElevator::get)
//                .whileTrue(elevator.setManualVoltage(0.5));
//        operatorDashboard.operatorKeypad.getOverride6()
//                .or(operatorDashboard.manualElevatorDown::get)
//                .and(operatorDashboard.manualElevator::get)
//                .whileTrue(elevator.setManualVoltage(-0.5));

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
