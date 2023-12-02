package frc.robot.commands.auto;

import static frc.constants.AutoConstants.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.constants.ManipulatorConstants;
import frc.robot.commands.CubeHuntCommand;
import frc.robot.commands.LogCommand;
import frc.robot.commands.PathCommand;
import frc.robot.location.AllianceFlipUtil;
import frc.robot.location.FieldPositions;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubystem;

// todo:
// speed up paths
// remove wait delays
// add event markers to lower wrist early??

public class RedThreeCubeShootLeftUseBlueInstead {

  public static Command get(DriveSubsystem driveSubsystem, ManipulatorSubystem manipulator) {

    final String AUTO_NAME = "R-11-3CubeShoot";
    final String PATH_NAME = "[RED]ThreeCubeStart"; // RED_CUBE_LEFT_PATH_NAME;
    PathPlannerTrajectory pathPlannerTrajectory =
        PathPlanner.loadPath(
            PATH_NAME, MAX_SPEED_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(
            // Shoot the cube
            manipulator.getShootCubeCommand(),
            manipulator.lowerPositionCommand(),
            new InstantCommand(
                () -> {
                  manipulator.release();
                  manipulator.rollerGrab();
                }),
            // new InstantCommand(() -> manipulator.grab()),
            // new InstantCommand(() -> manipulator.rollerDrop()),
            // new LogCommand("Started Rollers!!!!!!!!!!!!!!!!!!!!!!!!!!!!"),
            // new WaitCommand(2.0),
            // new InstantCommand(() -> manipulator.rollerStop()),
            // new LogCommand("Stopped Rollers!!!!!!!!!!!!!!!!!!!!!!!!!!!!"),

            new ProxyCommand(
                () ->
                    new FollowPathWithEvents(
                        new PathCommand(driveSubsystem, pathPlannerTrajectory, false, true),
                        pathPlannerTrajectory.getMarkers(),
                        manipulator.getEventMap())),
            new CubeHuntCommand(driveSubsystem, () -> manipulator.getRollerSensor()),
            new InstantCommand(manipulator::rollerStop),
            new InstantCommand(manipulator::grab),
            // new InstantCommand(driveSubsystem::stop),
            new InstantCommand(() -> manipulator.setWristReference(ManipulatorConstants.WRIST_MIN)),

            // move back to just inside community zone

            new ProxyCommand(() -> getDynamicPathToCommunityZone1(driveSubsystem)),
            new LogCommand("Finished return to CZ"),
            new InstantCommand(driveSubsystem::stop),
            // shoot the cube
            manipulator.getShootCubeCommand(),

            // new InstantCommand(() -> manipulator.rollerDrop()),
            // new WaitCommand(.25),
            // new InstantCommand(() -> manipulator.rollerStop()),

            // move to pick up next cube
            new InstantCommand(
                () -> manipulator.setWristReference(ManipulatorConstants.WRIST_LOWER)),
            // new WaitCommand(.5),
            new InstantCommand(manipulator::release),
            new InstantCommand(manipulator::rollerGrab),
            new ProxyCommand(() -> getDynamicPathToSecondCube(driveSubsystem)),
            new CubeHuntCommand(driveSubsystem, () -> manipulator.getRollerSensor()),
            new InstantCommand(manipulator::rollerStop),
            new InstantCommand(manipulator::grab),
            new InstantCommand(driveSubsystem::stop),
            new InstantCommand(() -> manipulator.setWristReference(ManipulatorConstants.WRIST_MIN)),

            // move back to just inside community zone
            new ProxyCommand(() -> getDynamicPathToTouchCommunityZone(driveSubsystem)),
            // shoot the cube
            manipulator.getShootCubeCommand(),
            // new InstantCommand(() -> manipulator.rollerDrop()),
            // new WaitCommand(.5),
            // new InstantCommand(() -> manipulator.rollerStop()),

            // move to pick up next cube

            new InstantCommand(
                () -> manipulator.setWristReference(ManipulatorConstants.WRIST_LOWER)),
            // new WaitCommand(.5),
            new InstantCommand(manipulator::release),
            new InstantCommand(manipulator::rollerGrab),
            new ProxyCommand(() -> getDynamicPathCommand4(driveSubsystem)),
            new CubeHuntCommand(driveSubsystem, () -> manipulator.getRollerSensor()),
            new InstantCommand(manipulator::rollerStop),
            new InstantCommand(manipulator::grab),
            new InstantCommand(driveSubsystem::stop),
            new InstantCommand(() -> manipulator.setWristReference(ManipulatorConstants.WRIST_MIN)),

            // move back to just inside community zone

            //   new ProxyCommand(() -> getDynamicPathCommand5(driveSubsystem)),
            new InstantCommand(driveSubsystem::stop),
            // shoot the cube
            //   new InstantCommand(manipulator::rollerDrop)
            //       .alongWith(
            //           new WaitCommand(.5)
            //               .andThen(new InstantCommand(() -> manipulator.rollerStop()))),
            new LogCommand("Finished with " + PATH_NAME))
        .withName(AUTO_NAME);
  }

  public static Command getDynamicPathToCommunityZone1(DriveSubsystem driveSubsystem) {
    Pose2d cubePose = new Pose2d(2.3, 4.42, Rotation2d.fromDegrees(180.0));

    PathPlannerTrajectory blueTrajectory =
        PathPlanner.generatePath(
            new PathConstraints(2.0, 4),
            new PathPoint(
                    driveSubsystem.getPose().getTranslation(),
                    Rotation2d.fromDegrees(-7.2),
                    driveSubsystem.getPose().getRotation())
                .withNextControlLength(1.0),
            new PathPoint(
                    cubePose.getTranslation(),
                    cubePose.getRotation(),
                    Rotation2d.fromDegrees(-180.0))
                .withPrevControlLength(1.0));

    PathPlannerTrajectory transformedTrajectory =
        PathPlannerTrajectory.transformTrajectoryForAlliance(
            blueTrajectory, DriverStation.getAlliance());

    Command command =
        new PathCommand( // 1 second
                driveSubsystem,
                transformedTrajectory,
                /*  PathPlanner.generatePath(
                new PathConstraints(2, 4),
                new PathPoint(
                        driveSubsystem.getPose().getTranslation(),
                        Rotation2d.fromDegrees(-7.2),
                        driveSubsystem.getPose().getRotation())
                    .withNextControlLength(1.0),
                new PathPoint(
                        new Translation2d(11.2, .93),
                        new Rotation2d(),
                        Rotation2d.fromDegrees(-180.0))
                    .withPrevControlLength(1.0)), */
                false,
                false)
            .beforeStarting(new LogCommand("Starting Path Command"))
            .andThen(new InstantCommand(driveSubsystem::stop))
            .andThen(new LogCommand("Finished PathCommand"));
    System.out.println("Making dynamic path command");
    return command;
  }

  public static Command getDynamicPathToSecondCube(DriveSubsystem driveSubsystem) {
    // Pose2d endPose =
    // LocationHelper.getPoseByDistanceAndAngleToPose(FieldPositions.blueStagingMark2, 1.2,
    // Rotation2d.fromDegrees(35.0));
    Pose2d endPose =
        LocationHelper.getPoseByDistanceAndAngleToPose(
            AllianceFlipUtil.apply(FieldPositions.blueStagingMark2),
            1.2,
            AllianceFlipUtil.apply(Rotation2d.fromDegrees(35.0)));

    Command command =
        new PathCommand(
                driveSubsystem,
                PathPlanner.generatePath(
                    new PathConstraints(2, 4),
                    new PathPoint(
                            driveSubsystem.getPose().getTranslation(),
                            Rotation2d.fromDegrees(-180.0),
                            driveSubsystem.getPose().getRotation())
                        .withNextControlLength(1.0),
                    new PathPoint(
                            endPose.getTranslation(),
                            // new Translation2d(10.5, 1.35),
                            Rotation2d.fromDegrees(145.0),
                            Rotation2d.fromDegrees(145.0))
                        .withNextControlLength(1.0)),
                false,
                false)
            .beforeStarting(new LogCommand("Starting second cube path Command"))
            .andThen(new InstantCommand(driveSubsystem::stop))
            .andThen(new LogCommand("Finished second cube path command"));
    System.out.println("Making dynamic second cube path command");

    return command;
  }

  public static Command getDynamicPathToTouchCommunityZone(DriveSubsystem driveSubsystem) {
    Command command =
        new PathCommand(
                driveSubsystem,
                PathPlanner.generatePath(
                    new PathConstraints(2, 4),
                    new PathPoint(
                            driveSubsystem.getPose().getTranslation(),
                            Rotation2d.fromDegrees(-48.0),
                            driveSubsystem.getPose().getRotation())
                        .withNextControlLength(1.0),
                    new PathPoint(
                            new Translation2d(11.2, 2.0),
                            Rotation2d.fromDegrees(11),
                            Rotation2d.fromDegrees(-180.0))
                        .withNextControlLength(1.0)),
                false,
                false)
            .beforeStarting(new LogCommand("Starting second return to CZ Command"))
            .andThen(new InstantCommand(driveSubsystem::stop))
            .andThen(new LogCommand("Finished second return to CZ command"));
    System.out.println("Making dynamic second return to CZ path command");

    return command;
  }

  public static Command getDynamicPathCommand4(DriveSubsystem driveSubsystem) {
    Command command =
        new PathCommand(
                driveSubsystem,
                PathPlanner.generatePath(
                    new PathConstraints(2, 4),
                    new PathPoint(
                            driveSubsystem.getPose().getTranslation(),
                            Rotation2d.fromDegrees(-180.0),
                            driveSubsystem.getPose().getRotation())
                        .withNextControlLength(1.0),
                    new PathPoint(
                            new Translation2d(10.05, 2.77),
                            Rotation2d.fromDegrees(135.0),
                            Rotation2d.fromDegrees(135.0))
                        .withNextControlLength(1.0)),
                false,
                false)
            .beforeStarting(new LogCommand("Starting third cube path Command"))
            .andThen(new InstantCommand(driveSubsystem::stop))
            .andThen(new LogCommand("Finished third cube path command"));
    System.out.println("Making dynamic third cube path command");

    return command;
  }

  public static Command getDynamicPathCommand5(DriveSubsystem driveSubsystem) {
    Command command =
        new PathCommand(
                driveSubsystem,
                PathPlanner.generatePath(
                    new PathConstraints(2, 4),
                    new PathPoint(
                            driveSubsystem.getPose().getTranslation(),
                            Rotation2d.fromDegrees(-74.63),
                            driveSubsystem.getPose().getRotation())
                        .withNextControlLength(1.0),
                    new PathPoint(
                            new Translation2d(11.53, .83),
                            Rotation2d.fromDegrees(-17.8),
                            Rotation2d.fromDegrees(-175.0))
                        .withPrevControlLength(.5)),
                false,
                false)
            .beforeStarting(new LogCommand("Starting third return to CZ Command"))
            .andThen(new InstantCommand(driveSubsystem::stop))
            .andThen(new LogCommand("Finished third return to CZ command"));
    System.out.println("Making dynamic third return to CZ path command");

    return command;
  }
}
