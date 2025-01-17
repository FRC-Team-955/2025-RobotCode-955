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

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.SubsystemBaseExt;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.lang.reflect.Array;
import java.util.HashSet;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private static final HashSet<SubsystemBaseExt> extendedSubsystems = new HashSet<>();
    private final RobotContainer robotContainer;
    private Command autonomousCommand;

    public static void registerExtendedSubsystem(SubsystemBaseExt subsystem) {
        if (!extendedSubsystems.add(subsystem)) {
            Util.error("An extended subsystem has been registered more than once: " + subsystem.getName());
        }
    }

//    private static void onCommandEnd(Command command) {
//        for (var subsystem : command.getRequirements()) {
//            if (subsystem instanceof SubsystemBaseExt) {
//                ((SubsystemBaseExt) subsystem).onCommandEnd();
//            } else {
//                Util.error("Subsystem " + subsystem.getName() + " is not an extended subsystem");
//            }
//        }
//    }

    public Robot() {
        AutoLogOutputManager.addPackage("frc");

        Logger.recordMetadata("* ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("* BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("* GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("* GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("* GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("* GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("* GitDirty", "Uncommitted changes");
                break;
            default:
                Logger.recordMetadata("* GitDirty", "Unknown");
                break;
        }
        logConstantClass(Constants.class, null);

        switch (Constants.mode) {
            case REAL -> {
                Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new NT4Publisher()); // Log to NetworkTables
                // SmartDashboard.putData("PowerDistribution", new PowerDistribution(Constants.pdhId, PowerDistribution.ModuleType.kRev)); // Enables power distribution logging
            }
            case SIM -> {
                Logger.addDataReceiver(new NT4Publisher());
            }
            case REPLAY -> {
                setUseTiming(!Constants.Simulation.replayRunAsFastAsPossible); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
                Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
                if (!Constants.Simulation.replayRunAsFastAsPossible)
                    Logger.addDataReceiver(new NT4Publisher()); // Log to NetworkTables if we are replaying in real time
            }
        }

        Logger.start();

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

//        CommandScheduler.getInstance().onCommandFinish(Robot::onCommandEnd);
//        CommandScheduler.getInstance().onCommandInterrupt(Robot::onCommandEnd);
    }

    private void logConstantClass(Class<?> clazz, String parentName) {
        var parent = (parentName != null ? parentName + "." : "");
        for (var field : clazz.getFields()) {
            var key = parent + clazz.getSimpleName() + "." + field.getName();
            try {
                var value = field.get(null);
                if (value.getClass().isArray()) {
                    for (int i = 0; i < Array.getLength(value); i++) {
                        Logger.recordMetadata(key + "[" + i + "]", Array.get(value, i).toString());
                    }
                } else {
                    Logger.recordMetadata(key, value.toString());
                }
            } catch (Throwable e) {
                Logger.recordMetadata(key, "Unknown");
            }
        }
        for (var subclass : clazz.getClasses()) {
            logConstantClass(subclass, parent + clazz.getSimpleName());
        }
    }

    /**
     * This function is called periodically during all modes.
     */
    @Override
    public void robotPeriodic() {
        // Switch thread to high priority to improve loop timing
        Threads.setCurrentThreadPriority(true, 99);

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();

        for (var subsystem : extendedSubsystems) {
            subsystem.periodicAfterCommands();
        }

        // Return to normal thread priority
        Threads.setCurrentThreadPriority(false, 10);
    }

    /**
     * This function is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {
    }

    /**
     * This function is called periodically when disabled.
     */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    /**
     * This function is called once when teleop is enabled.
     */
    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    /**
     * This function is called once when test mode is enabled.
     */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }
}
