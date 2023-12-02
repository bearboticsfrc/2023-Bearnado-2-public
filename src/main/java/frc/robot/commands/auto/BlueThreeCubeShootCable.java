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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.constants.ManipulatorConstants;
import frc.constants.VisionConstants;
import frc.robot.commands.CubeHuntCommand;
import frc.robot.commands.LogCommand;
import frc.robot.commands.PathCommand;
import frc.robot.fms.AllianceColor;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubystem;

// todo:
// speed up paths
// remove wait delays
// add event markers to lower wrist early??

public class BlueThreeCubeShootCable {

  public static Command get(DriveSubsystem driveSubsystem, ManipulatorSubystem manipulator) {

    final String AUTO_NAME = "B-11-3CubeShoot";
    final String PATH_NAME = "[BLUE]ThreeCubeStart"; // BLUE_CUBE_LEFT_PATH_NAME;
    PathPlannerTrajectory pathPlannerTrajectory =
        PathPlanner.loadPath(
            PATH_NAME, MAX_SPEED_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(
            // Shoot the cube
            manipulator.getShootCubeCommand(),
            manipulator.getWristFloorCommand(),
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
            new InstantCommand(manipulator::release),
            new InstantCommand(manipulator::rollerGrab),
            new CubeHuntCommand(driveSubsystem, () -> manipulator.getRollerSensor())
                .withTimeout(.5),
            new InstantCommand(manipulator::rollerStop),
            new InstantCommand(manipulator::grab),
            // new InstantCommand(driveSubsystem::stop),
            manipulator.getWristMinCommand(),

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
            manipulator.getWristFloorCommand(),
            // new WaitCommand(.5),
            new InstantCommand(manipulator::release),
            new InstantCommand(manipulator::rollerGrab),
            new ProxyCommand(() -> getDynamicPathToSecondCube(driveSubsystem)),
            new CubeHuntCommand(driveSubsystem, () -> manipulator.getRollerSensor())
                .withTimeout(.5),
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
            new ProxyCommand(() -> getDynamicPathToThirdCube(driveSubsystem)),
            new CubeHuntCommand(driveSubsystem, () -> manipulator.getRollerSensor())
                .withTimeout(0.5),
            new InstantCommand(manipulator::rollerStop),
            new InstantCommand(manipulator::grab),
            new InstantCommand(driveSubsystem::stop),
            new InstantCommand(() -> manipulator.setWristReference(ManipulatorConstants.WRIST_MIN)),

            // move back to just inside community zone

            new ProxyCommand(() -> getDynamicPathToTouchCommunityZoneFinal(driveSubsystem)),
            new InstantCommand(driveSubsystem::stop),

            // shoot the cube
            // new InstantCommand(manipulator::rollerDrop)
            //    .alongWith(
            //        new WaitCommand(.5)
            //            .andThen(new InstantCommand(() -> manipulator.rollerStop()))),
            new LogCommand("Finished with " + PATH_NAME))
        .withName(AUTO_NAME);
  }

  public static Pose2d transformYAxisForAllianceColor(Pose2d pose) {
    if (AllianceColor.alliance == Alliance.Blue) {
      return pose;
    }
    Translation2d transformedTranslation =
        new Translation2d(pose.getX(), VisionConstants.FIELD_WIDTH_METERS - pose.getY());
    Rotation2d transformedHolonomicRotation = pose.getRotation().times(-1);
    return new Pose2d(transformedTranslation, transformedHolonomicRotation);
  }

  public static Command getDynamicPathToCommunityZone1(DriveSubsystem driveSubsystem) {
    Pose2d startPose = driveSubsystem.getPose();
    Pose2d endPose =
        LocationHelper.transformYAxisForAllianceColor(
            new Pose2d(5.07, .93, Rotation2d.fromDegrees(180.0)));

    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(
            new PathConstraints(2.0, 4),
            new PathPoint(
                    startPose.getTranslation(),
                    Rotation2d.fromDegrees(0.0),
                    startPose.getRotation())
                .withNextControlLength(0.1),
            new PathPoint(
                    endPose.getTranslation(), endPose.getRotation(), Rotation2d.fromDegrees(0.0))
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
    Pose2d endPose =
        LocationHelper.transformYAxisForAllianceColor(
            new Pose2d(6.14, 1.45, Rotation2d.fromDegrees(35.0)));

    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(
            new PathConstraints(2.0, 4),
            new PathPoint(
                    driveSubsystem.getPose().getTranslation(),
                    Rotation2d.fromDegrees(0.0),
                    driveSubsystem.getPose().getRotation())
                .withNextControlLength(1.0),
            new PathPoint(endPose.getTranslation(), endPose.getRotation(), endPose.getRotation())
                .withPrevControlLength(1.0));

    Command command =
        new PathCommand( // 1 second
                driveSubsystem, trajectory, false, false)
            .beforeStarting(new LogCommand("Starting second cube path Command"))
            .andThen(new InstantCommand(driveSubsystem::stop))
            .andThen(new LogCommand("Finished second cube path command"));
    System.out.println("Making dynamic second cube path command");

    return command;
  }

  public static Command getDynamicPathToTouchCommunityZone(DriveSubsystem driveSubsystem) {
    Pose2d endPose =
        LocationHelper.transformYAxisForAllianceColor(
            new Pose2d(5.16, .9, Rotation2d.fromDegrees(0.0)));
    //   new Pose2d(5.26, 1.5, Rotation2d.fromDegrees(0.0)));

    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(
            new PathConstraints(2.0, 4),
            new PathPoint(
                    driveSubsystem.getPose().getTranslation(),
                    Rotation2d.fromDegrees(180),
                    driveSubsystem.getPose().getRotation())
                .withNextControlLength(0.4),
            new PathPoint(
                    endPose.getTranslation(), Rotation2d.fromDegrees(180), endPose.getRotation())
                .withPrevControlLength(0.4));

    Command command =
        new PathCommand(driveSubsystem, trajectory, false, false)
            .beforeStarting(new LogCommand("Starting second return to CZ Command"))
            .andThen(new InstantCommand(driveSubsystem::stop))
            .andThen(new LogCommand("Finished second return to CZ command"));
    System.out.println("Making dynamic second return to CZ path command");

    return command;
  }

  public static Command getDynamicPathToThirdCube(DriveSubsystem driveSubsystem) {
    // Pose[6.27,2.51@45.00]
    Pose2d endPose =
        LocationHelper.transformYAxisForAllianceColor(
            new Pose2d(6.27, 2.51, Rotation2d.fromDegrees(45.0)));

    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(
            new PathConstraints(2.0, 4),
            new PathPoint(
                    driveSubsystem.getPose().getTranslation(),
                    LocationHelper.transformHeadingForAllianceColor(Rotation2d.fromDegrees(45.0)),
                    driveSubsystem.getPose().getRotation())
                .withNextControlLength(.4),
            new PathPoint(endPose.getTranslation(), endPose.getRotation(), endPose.getRotation())
                .withPrevControlLength(.4));

    Command command =
        new PathCommand(driveSubsystem, trajectory, false, false)
            .beforeStarting(new LogCommand("Starting third cube path Command"))
            .andThen(new InstantCommand(driveSubsystem::stop))
            .andThen(new LogCommand("Finished third cube path command"));
    System.out.println("Making dynamic third cube path command");

    return command;
  }

  public static Command getDynamicPathToTouchCommunityZoneFinal(DriveSubsystem driveSubsystem) {
    Pose2d endPose =
        LocationHelper.transformYAxisForAllianceColor(
            new Pose2d(5.0, .70, Rotation2d.fromDegrees(180.0)));

    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(
            new PathConstraints(2.5, 4),
            new PathPoint(
                    driveSubsystem.getPose().getTranslation(),
                    Rotation2d.fromDegrees(180.0),
                    driveSubsystem.getPose().getRotation())
                .withNextControlLength(0.4),
            new PathPoint(
                    endPose.getTranslation(), endPose.getRotation(), Rotation2d.fromDegrees(0.0))
                .withPrevControlLength(0.4));

    Command command =
        new PathCommand( // 1 second
                driveSubsystem, trajectory, false, false)
            .beforeStarting(new LogCommand("Starting third return to CZ Command"))
            .andThen(new InstantCommand(driveSubsystem::stop))
            .andThen(new LogCommand("Finished third return to CZ command"));
    System.out.println("Making dynamic third return to CZ path command");

    return command;
  }
}
