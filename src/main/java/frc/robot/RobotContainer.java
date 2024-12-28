package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.CommandNintendoSwitchProController;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final CommandXboxController driverController = Constants.Simulation.useNintendoSwitchProController ? new CommandNintendoSwitchProController(0) : new CommandXboxController(0);

    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    private final LoggedDashboardChooser<Command> characterizationChooser = new LoggedDashboardChooser<>("Characterization Choices");

    private final RobotState robotState = RobotState.get();

    /* Subsystems */
    private final Drive drive = Drive.get();
    private final Intake intake = Intake.get();
    private final Shooter shooter = Shooter.get();

    public RobotContainer() {
        addAutos();
        addCharacterizations();
        configureButtonBindings();

        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    }

    private void addAutos() {
        autoChooser.addOption("None", null);

        var factory = drive.createAutoFactory(new AutoFactory.AutoBindings());

        autoChooser.addOption("Characterization", Commands.deferredProxy(characterizationChooser::get));
    }

    private void addCharacterizations() {
        ////////////////////// DRIVE //////////////////////

        characterizationChooser.addOption(
                "Drive Wheel Radius (Clockwise)",
                drive.wheelRadiusCharacterization(Drive.WheelRadiusCharacterization.Direction.CLOCKWISE)
        );
        characterizationChooser.addOption(
                "Drive Wheel Radius (Counter-Clockwise)",
                drive.wheelRadiusCharacterization(Drive.WheelRadiusCharacterization.Direction.COUNTER_CLOCKWISE)
        );

        characterizationChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysId.quasistatic(SysIdRoutine.Direction.kForward)
        );
        characterizationChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysId.quasistatic(SysIdRoutine.Direction.kReverse)
        );
        characterizationChooser.addOption(
                "Drive SysId (Dynamic Forward)",
                drive.sysId.dynamic(SysIdRoutine.Direction.kForward)
        );
        characterizationChooser.addOption(
                "Drive SysId (Dynamic Reverse)",
                drive.sysId.dynamic(SysIdRoutine.Direction.kReverse)
        );

        ////////////////////// SHOOTER //////////////////////

        characterizationChooser.addOption(
                "Shooter Pivot SysId (Quasistatic Forward)",
                shooter.pivotSysId.quasistatic(SysIdRoutine.Direction.kForward)
        );
        characterizationChooser.addOption(
                "Shooter Pivot SysId (Quasistatic Reverse)",
                shooter.pivotSysId.quasistatic(SysIdRoutine.Direction.kReverse)
        );
        characterizationChooser.addOption(
                "Shooter Pivot SysId (Dynamic Forward)",
                shooter.pivotSysId.dynamic(SysIdRoutine.Direction.kForward)
        );
        characterizationChooser.addOption(
                "Shooter Pivot SysId (Dynamic Reverse)",
                shooter.pivotSysId.dynamic(SysIdRoutine.Direction.kReverse)
        );

        characterizationChooser.addOption(
                "Shooter Feed SysId (Quasistatic Forward)",
                shooter.feedSysId.quasistatic(SysIdRoutine.Direction.kForward)
        );
        characterizationChooser.addOption(
                "Shooter Feed SysId (Quasistatic Reverse)",
                shooter.feedSysId.quasistatic(SysIdRoutine.Direction.kReverse)
        );
        characterizationChooser.addOption(
                "Shooter Feed SysId (Dynamic Forward)",
                shooter.feedSysId.dynamic(SysIdRoutine.Direction.kForward)
        );
        characterizationChooser.addOption(
                "Shooter Feed SysId (Dynamic Reverse)",
                shooter.feedSysId.dynamic(SysIdRoutine.Direction.kReverse)
        );

        characterizationChooser.addOption(
                "Shooter Flywheels SysId (Quasistatic Forward)",
                shooter.flywheelsSysId.quasistatic(SysIdRoutine.Direction.kForward)
        );
        characterizationChooser.addOption(
                "Shooter Flywheels SysId (Quasistatic Reverse)",
                shooter.flywheelsSysId.quasistatic(SysIdRoutine.Direction.kReverse)
        );
        characterizationChooser.addOption(
                "Shooter Flywheels SysId (Dynamic Forward)",
                shooter.flywheelsSysId.dynamic(SysIdRoutine.Direction.kForward)
        );
        characterizationChooser.addOption(
                "Shooter Flywheels SysId (Dynamic Reverse)",
                shooter.flywheelsSysId.dynamic(SysIdRoutine.Direction.kReverse)
        );

        ////////////////////// INTAKE //////////////////////

        characterizationChooser.addOption(
                "Intake Pivot SysId (Quasistatic Forward)",
                intake.pivotSysId.quasistatic(SysIdRoutine.Direction.kForward)
        );
        characterizationChooser.addOption(
                "Intake Pivot SysId (Quasistatic Reverse)",
                intake.pivotSysId.quasistatic(SysIdRoutine.Direction.kReverse)
        );
        characterizationChooser.addOption(
                "Intake Pivot SysId (Dynamic Forward)",
                intake.pivotSysId.dynamic(SysIdRoutine.Direction.kForward)
        );
        characterizationChooser.addOption(
                "Intake Pivot SysId (Dynamic Reverse)",
                intake.pivotSysId.dynamic(SysIdRoutine.Direction.kReverse)
        );

        characterizationChooser.addOption(
                "Intake Feed SysId (Quasistatic Forward)",
                intake.feedSysId.quasistatic(SysIdRoutine.Direction.kForward)
        );
        characterizationChooser.addOption(
                "Intake Feed SysId (Quasistatic Reverse)",
                intake.feedSysId.quasistatic(SysIdRoutine.Direction.kReverse)
        );
        characterizationChooser.addOption(
                "Intake Feed SysId (Dynamic Forward)",
                intake.feedSysId.dynamic(SysIdRoutine.Direction.kForward)
        );
        characterizationChooser.addOption(
                "Intake Feed SysId (Dynamic Reverse)",
                intake.feedSysId.dynamic(SysIdRoutine.Direction.kReverse)
        );
    }

    private void configureButtonBindings() {
        driverController.y().onTrue(robotState.resetRotation());

        //noinspection SuspiciousNameCombination
        RobotModeTriggers.teleop().whileTrue(
                drive.driveJoystick(
                        driverController::getLeftY,
                        driverController::getLeftX,
                        () -> -driverController.getRightX()
                )
        );
    }

    public Command getAutonomousCommand() {
        // Return null to do nothing.
        return autoChooser.get();
    }
}
