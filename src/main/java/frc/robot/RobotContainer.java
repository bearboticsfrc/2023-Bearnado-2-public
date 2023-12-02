// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.OIConstants;
import frc.robot.commands.AlignGridCommand;
import frc.robot.commands.DriveToNearestPoseCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.TurnCommand;
import frc.robot.commands.auto.AnyCubeOnly;
import frc.robot.commands.auto.AnyLeaveCommunity;
import frc.robot.commands.auto.BlueCubeCubeGrabEngageSubstation;
import frc.robot.commands.auto.BlueCubeCubeGrabSubstation;
import frc.robot.commands.auto.BlueCubeCubeLeft;
import frc.robot.commands.auto.BlueCubeLeft;
import frc.robot.commands.auto.BlueCubeLeftCS;
import frc.robot.commands.auto.BlueCubeRight;
import frc.robot.commands.auto.BlueCubeRightCS;
import frc.robot.commands.auto.BlueThreeCubeShootCable;
import frc.robot.commands.auto.HighConeScore;
import frc.robot.commands.auto.MidConeScore;
import frc.robot.commands.auto.MiddleCubeEngage;
import frc.robot.fms.AllianceColor;
import frc.robot.location.FieldPositions;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ManipulatorSubystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import java.util.ArrayList;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private final LedSubsystem ledSubsystem = new LedSubsystem();
  private final PneumaticHub pneumatichub = new PneumaticHub(30);
  private final ManipulatorSubystem manipulator = new ManipulatorSubystem(pneumatichub);
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;
  private final DigitalInput colorSwitch = new DigitalInput(0);

  private boolean isTeleop = false;

  CommandXboxController driveXboxController =
      new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
  CommandXboxController operatorXboxController =
      new CommandXboxController(OIConstants.OPERATOR_CONTROLLER_PORT);
  GenericHID gridSelector = new GenericHID(OIConstants.MACROPAD_CONTROLLER_PORT);

  // private List<Command> autonomousList = new ArrayList<Command>();
  private List<Pair<String, Command>> autoList = new ArrayList<Pair<String, Command>>();

  // A chooser for autonomous commands
  private SendableChooser<Command> chooser = new SendableChooser<>();

  private final DriveToPoseCommand driveToPoseCommand;
  private final DriveToNearestPoseCommand driveToNearestPoseCommand;
  private final FieldPositions fieldPositions;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    pneumatichub.enableCompressorDigital();

    fieldPositions = new FieldPositions();

    poseEstimatorSubsystem = new PoseEstimatorSubsystem(robotDrive, fieldPositions);

    setDefaultCommand();
    buildAutoList();
    setupShuffleboardTab();

    driveToPoseCommand = new DriveToPoseCommand(robotDrive, poseEstimatorSubsystem);
    driveToNearestPoseCommand =
        new DriveToNearestPoseCommand(robotDrive, poseEstimatorSubsystem, fieldPositions);
  }

  public void setFlashyLEDMode() {
    ledSubsystem.runDefaultCommand(colorSwitch.get());
  }

  public void autonomousInit() {
    isTeleop = false;
    robotDrive.setParkMode(false);
    ledSubsystem.setFastLarsonAnimation(AllianceColor.alliance == Alliance.Red);
  }

  public void teleopInit() {
    isTeleop = true;
    robotDrive.setParkMode(false);
    ledSubsystem.turnOff();
  }

  public void disabledInit() {
    isTeleop = false;
  }

  public DriveSubsystem getDriveSubsystem() {
    return robotDrive;
  }

  private void setupShuffleboardTab() {
    ShuffleboardTab compTab = Shuffleboard.getTab("Competition");
    for (Pair<String, Command> command : autoList) {
      chooser.addOption(command.getFirst(), command.getSecond());
    }

    // for (Command command : autonomousList) {
    //  chooser.addOption(command.getName(), command);
    // }
    chooser.setDefaultOption(autoList.get(0).getFirst(), autoList.get(0).getSecond());
    compTab.add("Auto Command", chooser).withSize(4, 1).withPosition(0, 1);
  }

  private void addToAutoList(String name, Command command) {
    autoList.add(new Pair<String, Command>(name, command));
  }

  private void buildAutoList() {
    addToAutoList("0-Nothing", new InstantCommand().withName("Nothing"));
    addToAutoList("1-LeaveCommunity", AnyLeaveCommunity.get(robotDrive));
    addToAutoList("2-Any-Cube-NoMove", AnyCubeOnly.get(robotDrive, manipulator));
    addToAutoList("3-Cube-Grab-Substation", BlueCubeLeft.get(robotDrive, manipulator));
    addToAutoList("4-Cube-Engage-Substation", BlueCubeLeftCS.get(robotDrive, manipulator));

    addToAutoList("5-Cube-Cube-SubStation", BlueCubeCubeLeft.get(robotDrive, manipulator, false));
    addToAutoList(
        "6-Cube-Cube-Engage-SubStation", BlueCubeCubeLeft.get(robotDrive, manipulator, true));
    addToAutoList(
        "7-Cube-Cube-Style-Substation", BlueCubeCubeGrabSubstation.get(robotDrive, manipulator));
    addToAutoList(
        "8-Cube-Cube-Grab-Engage-Substation",
        BlueCubeCubeGrabEngageSubstation.get(robotDrive, manipulator, true));

    addToAutoList("9-Cube-Grab-Cable", BlueCubeRight.get(robotDrive, manipulator));
    addToAutoList("10-Cube-Engage-Cable", BlueCubeRightCS.get(robotDrive, manipulator));

    addToAutoList(
        "11-Three-Cube-Shoot-Cable", BlueThreeCubeShootCable.get(robotDrive, manipulator));

    addToAutoList("12-Cube-Engage-Middle", MiddleCubeEngage.get(robotDrive, manipulator));

    addToAutoList("13-Cone-Cube-Substation", MidConeScore.get(robotDrive, manipulator, false));
    addToAutoList(
        "14-Cone-Cube-Engage-Substation", MidConeScore.get(robotDrive, manipulator, true));
    addToAutoList("15-Cone-High", HighConeScore.get(robotDrive, manipulator, false));
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  private void setDefaultCommand() {
    robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                robotDrive.drive(
                    -MathUtil.applyDeadband(driveXboxController.getLeftY(), 0.1),
                    -MathUtil.applyDeadband(driveXboxController.getLeftX(), 0.1),
                    -MathUtil.applyDeadband(driveXboxController.getRightX(), 0.1)),
            robotDrive));

    manipulator.setDefaultCommand(
        new RunCommand(
            () ->
                manipulator.runRollerDefault(
                    MathUtil.applyDeadband(operatorXboxController.getLeftTriggerAxis(), 0.1),
                    MathUtil.applyDeadband(operatorXboxController.getRightTriggerAxis(), 0.1)),
            manipulator));
  }

  public void configureControllerMappings() {
    configureDriverController(driveXboxController);
    configureOperatorController(operatorXboxController);
  }

  private void configureDriverController(CommandXboxController controller) {
    controller.y().whileTrue(manipulator.getIntakeObjectCommand(robotDrive));
    controller.x().whileTrue(new AlignGridCommand(robotDrive));
    controller
        .b()
        .onTrue(new InstantCommand(() -> robotDrive.setParkMode(true)))
        .onFalse(new InstantCommand(() -> robotDrive.setParkMode(false)));
    controller.a().onTrue(new InstantCommand(robotDrive::zeroHeading));

    controller
        .povUp()
        .whileTrue(
            new TurnCommand(
                    robotDrive,
                    () -> -MathUtil.applyDeadband(driveXboxController.getLeftY(), 0.1),
                    () -> -MathUtil.applyDeadband(driveXboxController.getLeftX(), 0.1),
                    0)
                .withTimeout(2.0));

    controller
        .povDown()
        .whileTrue(
            new TurnCommand(
                    robotDrive,
                    () -> -MathUtil.applyDeadband(driveXboxController.getLeftY(), 0.1),
                    () -> -MathUtil.applyDeadband(driveXboxController.getLeftX(), 0.1),
                    180)
                .withTimeout(2.0));

    controller
        .leftBumper()
        .whileTrue(
            new StartEndCommand(
                () -> robotDrive.setFieldRelative(false), () -> robotDrive.setFieldRelative(true)));
    controller
        .rightBumper()
        .whileTrue(
            manipulator
                .getMidElevatorCommand()
                .andThen(driveToNearestPoseCommand.get(fieldPositions.getSubstationList())));

    controller
        .leftTrigger(0.5)
        .onTrue(new InstantCommand(() -> robotDrive.setTurboMode(true)))
        .onFalse(new InstantCommand(() -> robotDrive.setTurboMode(false)));

    controller
        .rightTrigger(0.5)
        .onTrue(new InstantCommand(() -> robotDrive.setTurtleMode(true)))
        .onFalse(new InstantCommand(() -> robotDrive.setTurtleMode(false)));

    new Trigger(() -> hasGameObject() && isTeleop)
        .onTrue(
            new InstantCommand(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 1.0))
                .andThen(new WaitCommand(1.0))
                .andThen(
                    new InstantCommand(
                        () -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0))));
  }

  public void configureOperatorController(CommandXboxController controller) {
    controller
        .start()
        .toggleOnTrue(new StartEndCommand(ledSubsystem::setColorYellow, ledSubsystem::turnOff));
    controller
        .back()
        .toggleOnTrue(new StartEndCommand(ledSubsystem::setColorPurple, ledSubsystem::turnOff));

    controller.x().onTrue(manipulator.upperConePositionCommand());
    controller.y().onTrue(manipulator.getHomeCommand());
    controller.a().onTrue(manipulator.getFloorPickupPositionCommand());
    controller.b().onTrue(manipulator.getShelfPickupCommand());

    controller.povUp().onTrue(manipulator.upperCubePositionCommand());
    controller.povLeft().onTrue(manipulator.middleCubePositionCommand());
    controller.povRight().onTrue(manipulator.middleConePositionCommand());
    controller.povDown().onTrue(manipulator.lowerPositionCommand());

    // controller.leftBumper().whileTrue(manipulator.getMidElevatorCommand());

    controller
        .rightBumper()
        .toggleOnTrue(new StartEndCommand(manipulator::grab, manipulator::release));
  }

  public boolean hasGameObject() {
    return manipulator.getRollerSensor() < 2.5;
  }
}
