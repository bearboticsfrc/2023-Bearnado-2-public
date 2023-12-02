package frc.robot.commands.auto;

import static frc.constants.AutoConstants.*;
import static frc.constants.ManipulatorConstants.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.CubeHuntCommand;
import frc.robot.commands.LogCommand;
import frc.robot.commands.PathCommand;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubystem;

public class BlueCubeCubeGrabEngageSubstation {

  public static Command get(
      DriveSubsystem driveSubsystem, ManipulatorSubystem manipulator, boolean climb) {

    final String PATH_NAME = BLUE_CUBE_LEFT_PATH_NAME;

    PathPlannerTrajectory pathPlannerTrajectory =
        PathPlanner.loadPath(PATH_NAME, 2.5, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(
            manipulator.getShootCubeCommand(),
            new ProxyCommand(
                () ->
                    new FollowPathWithEvents(
                        new PathCommand(driveSubsystem, pathPlannerTrajectory, false, true),
                        pathPlannerTrajectory.getMarkers(),
                        manipulator.getEventMap())),
            new InstantCommand(() -> manipulator.release()),
            new InstantCommand(manipulator::rollerGrab),
            new LogCommand("Starting cube hunt ..."),
            new CubeHuntCommand(driveSubsystem, () -> manipulator.getRollerSensor())
                .withTimeout(0.5),
            new LogCommand("Finnishing cube hunt ..."),
            new InstantCommand(
                () -> {
                  manipulator.rollerStop();
                  manipulator.grab();
                  driveSubsystem.stop();
                }),
            manipulator.setWristCommand(WRIST_SAFE_MIN + .05),
            new WaitUntilCommand(() -> manipulator.isWristInSafeZone()),
            manipulator.setWristCommand(WRIST_MIN),
            new ProxyCommand(() -> getDynamicPathToCubeNode(driveSubsystem)),
            new InstantCommand(driveSubsystem::stop),
            // shoot the cube to mid
            new InstantCommand(manipulator::rollerDropSlower),
            new WaitCommand(.1),
            new InstantCommand(manipulator::rollerStop),
            new InstantCommand(manipulator::release),
            manipulator.setWristCommand(WRIST_30_UP),
            new ProxyCommand(() -> getDynamicPathToSecondCube(driveSubsystem)),
            manipulator.getWristFloorCommand(),
            new InstantCommand(manipulator::release),
            new InstantCommand(manipulator::rollerGrab),
            new CubeHuntCommand(driveSubsystem, () -> manipulator.getRollerSensor())
                .withTimeout(.5),
            new LogCommand("Finnishing cube hunt ..."),
            new InstantCommand(
                () -> {
                  manipulator.rollerStop();
                  manipulator.grab();
                  driveSubsystem.stop();
                }),
            manipulator.setWristCommand(WRIST_SAFE_MIN + .05),
            new ConditionalCommand(
                new ProxyCommand(() -> getDynamicPathToClimbFromOutside(driveSubsystem))
                    .andThen(
                        new AutoBalanceCommand(driveSubsystem)
                            .alongWith(
                                new WaitUntilCommand(14.9)
                                    .andThen(
                                        new InstantCommand(
                                            () -> driveSubsystem.setParkMode(true))))),
                new InstantCommand(),
                () -> climb),
            manipulator.setWristCommand(WRIST_MIN),
            new InstantCommand(manipulator::rollerDrop),
            new WaitCommand(.1),
            new InstantCommand(manipulator::rollerStop),
            new InstantCommand(manipulator::release))
        .withName("BlueCubeCubeGrabEngageSubstation");
  }

  public static Command getDynamicPathToCubeNode(DriveSubsystem driveSubsystem) {
    // Pose2d cubeNodePose = new Pose2d(1.9, 1.05, new Rotation2d());
    // Pose2d cubeNodePose = new Pose2d(1.9, 4.42, Rotation2d.fromDegrees(180.0));
    Pose2d cubeNodePose =
        LocationHelper.transformYAxisForAllianceColor(
            new Pose2d(2.3, 4.52, Rotation2d.fromDegrees(180.0)));
    Pose2d entryPose =
        LocationHelper.transformYAxisForAllianceColor(
            new Pose2d(4.0, 4.85, Rotation2d.fromDegrees(180.0)));

    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(
            new PathConstraints(2.5, 4),
            new PathPoint(
                    driveSubsystem.getPose().getTranslation(),
                    LocationHelper.transformHeadingForAllianceColor(Rotation2d.fromDegrees(155.0)),
                    driveSubsystem.getPose().getRotation())
                .withNextControlLength(1.0),
            new PathPoint(
                    entryPose.getTranslation(),
                    entryPose.getRotation(),
                    Rotation2d.fromDegrees(0.0))
                .withPrevControlLength(1.0),
            new PathPoint(
                    cubeNodePose.getTranslation(),
                    cubeNodePose.getRotation(),
                    Rotation2d.fromDegrees(0.0))
                .withPrevControlLength(1.0));

    Command command =
        new PathCommand(driveSubsystem, trajectory, false, false)
            .beforeStarting(new LogCommand("Starting Path Command"))
            .andThen(new InstantCommand(driveSubsystem::stop))
            .andThen(new LogCommand("Finished PathCommand"));
    System.out.println("Making dynamic path command");
    return command;
  }

  public static Command getDynamicPathToSecondCube(DriveSubsystem driveSubsystem) {
    // Pose2d cubeNodePose = new Pose2d(1.9, 1.05, new Rotation2d());
    // Pose2d cubeNodePose = new Pose2d(1.9, 4.42, Rotation2d.fromDegrees(180.0));
    Pose2d cubeNodePose =
        LocationHelper.transformYAxisForAllianceColor(
            new Pose2d(6.0, 4.50, Rotation2d.fromDegrees(-45.0)));
    //            new Pose2d(6.27, 4.21, Rotation2d.fromDegrees(-45.0)));
    // Pose2d entryPose =
    //     LocationHelper.transformYAxisForAllianceColor(
    //         new Pose2d(4.0, 4.85, Rotation2d.fromDegrees(180.0)));

    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(
            new PathConstraints(2.5, 4),
            new PathPoint(
                    driveSubsystem.getPose().getTranslation(),
                    LocationHelper.transformHeadingForAllianceColor(Rotation2d.fromDegrees(20.0)),
                    driveSubsystem.getPose().getRotation())
                .withNextControlLength(.75),
            new PathPoint(
                    cubeNodePose.getTranslation(),
                    cubeNodePose.getRotation(),
                    cubeNodePose.getRotation())
                .withPrevControlLength(1.0));

    Command command =
        new PathCommand(driveSubsystem, trajectory, false, false)
            .beforeStarting(new LogCommand("Starting Path Command"))
            .andThen(new InstantCommand(driveSubsystem::stop))
            .andThen(new LogCommand("Finished PathCommand"));
    System.out.println("Making dynamic path command");
    return command;
  }

  /*
   * Make a dynamic path to center of community near the grid, then drive up the charge station.   Assumes robot is on the substation side.
   *
   */
  public static Command getDynamicPathToClimbFromOutside(DriveSubsystem driveSubsystem) {

    Pose2d entryPose =
        LocationHelper.transformYAxisForAllianceColor(
            new Pose2d(5.83, 2.75, Rotation2d.fromDegrees(0.0)));

    Pose2d chargeStationPose =
        LocationHelper.transformYAxisForAllianceColor(
            new Pose2d(3.0, 2.75, Rotation2d.fromDegrees(0.0)));

    PathPlannerTrajectory blueTrajectory =
        PathPlanner.generatePath(
            new PathConstraints(2.25, 4),
            new PathPoint(
                    driveSubsystem.getPose().getTranslation(),
                    LocationHelper.transformHeadingForAllianceColor(Rotation2d.fromDegrees(-45.0)),
                    driveSubsystem.getPose().getRotation())
                .withNextControlLength(1.0),
            new PathPoint(
                    entryPose.getTranslation(),
                    Rotation2d.fromDegrees(180.0),
                    Rotation2d.fromDegrees(0.0))
                .withControlLengths(0.75, 0.75),
            new PathPoint(
                    chargeStationPose.getTranslation(),
                    Rotation2d.fromDegrees(180.0),
                    Rotation2d.fromDegrees(0.0))
                .withPrevControlLength(1.0));

    Command command =
        new PathCommand(driveSubsystem, blueTrajectory, false, false)
            .beforeStarting(new LogCommand("Starting Path Command"))
            .andThen(new InstantCommand(driveSubsystem::stop))
            .andThen(new LogCommand("Finished PathCommand"));
    System.out.println("Making dynamic path command");
    return command;
  }
}
