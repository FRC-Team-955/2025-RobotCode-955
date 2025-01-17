// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Drive;
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

    public RobotContainer() {
        addAutos();
        addCharacterizations();
        configureButtonBindings();

        // Configure the button bindings
        configureButtonBindings();
    }

    private void addAutos() {
    }

    private void addCharacterizations() {
        ////////////////////// DRIVE //////////////////////

        characterizationChooser.addOption("Drive Wheel Radius Characterization", drive.wheelRadiusCharacterization(Drive.WheelRadiusCharacterization.Direction.CLOCKWISE));
        characterizationChooser.addOption("Drive SysId (Quasistatic Forward)", drive.sysId.quasistatic(SysIdRoutine.Direction.kForward));
        characterizationChooser.addOption("Drive SysId (Quasistatic Reverse)", drive.sysId.quasistatic(SysIdRoutine.Direction.kReverse));
        characterizationChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysId.dynamic(SysIdRoutine.Direction.kForward));
        characterizationChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysId.dynamic(SysIdRoutine.Direction.kReverse));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        driverController.y().onTrue(robotState.resetRotation());

        //noinspection SuspiciousNameCombination
        drive.setDefaultCommand(
                drive.driveJoystick(
                        driverController::getLeftY,
                        driverController::getLeftX,
                        () -> -driverController.getRightX()
                )
        );

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
