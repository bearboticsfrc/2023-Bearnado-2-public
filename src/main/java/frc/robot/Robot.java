// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.fms.AllianceColor;
import frc.robot.util.GitVersion;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;

  private StringLogEntry commandInitializeLogEntry;
  private StringLogEntry commandInterruptedLogEntry;
  private StringLogEntry commandFinishedLogEntry;

  /** Hold a reference to the last instance for a psuedo singleton pattern usage */
  public static Robot instance = null;

  public Robot() {
    instance = this;
  }

  /** Static method that returns true if the robot is currently in autonomous mode */
  public static boolean inAuto() {
    if (instance == null) return false;
    return instance.isAutonomous();
  }

  /** Static method that returns true if the robot is enabled */
  public static boolean isRobotEnabled() {
    if (instance == null) return false;
    return instance.isEnabled();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  Alliance firstAlliance = Alliance.Invalid;

  public void checkDriverStationUpdate() {
    Alliance alliance = DriverStation.getAlliance();

    if (DriverStation.isDSAttached() && alliance != firstAlliance) {
      AllianceColor.setAllianceColor(alliance);
      firstAlliance = alliance;
    }
  }

  @Override
  public void robotInit() {
    DriverStation.silenceJoystickConnectionWarning(true);

    // Enable this line to display a camera feed from a usb camera plugged into the roborio
    // CameraServer.startAutomaticCapture();

    DataLogManager.start();
    DataLogManager.log("Logging Started.");
    DataLogManager.logNetworkTables(true);
    DataLog log = DataLogManager.getLog();
    DriverStation.startDataLog(log);

    GitVersion version = GitVersion.loadVersion();
    version.printVersions();

    checkDriverStationUpdate();

    // Enable this line for debugging pathplanner paths, will display current path in the
    // PathPlanner desktop app.  The parameter is the port number.
    // PathPlannerServer.startServer(5811);

    robotContainer = new RobotContainer();
    robotContainer.configureControllerMappings();

    // Set the scheduler to log Shuffleboard events for command initialize, interrupt, finish
    commandInitializeLogEntry = new StringLogEntry(log, "/commands/initialized");
    commandInterruptedLogEntry = new StringLogEntry(log, "/commands/interrupted");
    commandFinishedLogEntry = new StringLogEntry(log, "/commands/finished");
    CommandScheduler.getInstance()
        .onCommandInitialize(command -> commandInitializeLogEntry.append(command.getName()));

    CommandScheduler.getInstance()
        .onCommandInterrupt(command -> commandInterruptedLogEntry.append(command.getName()));

    CommandScheduler.getInstance()
        .onCommandFinish(command -> commandFinishedLogEntry.append(command.getName()));
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands.
    checkDriverStationUpdate();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    robotContainer.autonomousInit();
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    checkDriverStationUpdate();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    robotContainer.teleopInit();
  }

  @Override
  public void disabledInit() {
    robotContainer.getDriveSubsystem().setParkMode(true);
    robotContainer.setFlashyLEDMode();
    robotContainer.disabledInit();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
