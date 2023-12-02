package frc.robot.commands.auto;

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

public class MidConeScore {
  public static Command get(
      DriveSubsystem driveSubsystem, ManipulatorSubystem manipulator, boolean climb) {

    // final String PATH_NAME = BLUE_CONE_SUBSTATION;
    final String PATH_NAME = "[BLUE]Cone-SubCopy";

    // PathPlannerTrajectory pathPlannerTrajectory = PathPlanner.loadPath(PATH_NAME, 1.6, .8);
    PathPlannerTrajectory pathPlannerTrajectory = PathPlanner.loadPath(PATH_NAME, 2.0, 2.0);

    return new SequentialCommandGroup(
            manipulator.getPositionAndScoreMiddleCommand(),
            new FollowPathWithEvents(
                new PathCommand(driveSubsystem, pathPlannerTrajectory, true, true),
                pathPlannerTrajectory.getMarkers(),
                manipulator.getEventMap()),
            // manipulator.getFloorPickupPositionCommand(),
            new InstantCommand(manipulator::rollerGrab),
            new WaitUntilCommand(manipulator::isWristAtSetPoint),
            new CubeHuntCommand(driveSubsystem, () -> manipulator.getRollerSensor())
                .withTimeout(.5),
            new InstantCommand(manipulator::rollerStop),
            new InstantCommand(manipulator::grab),
            new InstantCommand(driveSubsystem::stop),
            // set wrist to min
            manipulator.getWristMinCommand(),
            new ProxyCommand(() -> getDynamicPathToCubeNode(driveSubsystem)),
            new InstantCommand(driveSubsystem::stop),
            // shoot the cube to mid
            new InstantCommand(manipulator::rollerDropSlower),
            new WaitCommand(.2),
            new InstantCommand(manipulator::rollerStop),
            new InstantCommand(manipulator::release),
            new ConditionalCommand(
                new ProxyCommand(() -> AutoCommon.getDynamicPathToClimb(driveSubsystem))
                    .andThen(
                        new AutoBalanceCommand(driveSubsystem)
                            .alongWith(
                                new WaitUntilCommand(14.8)
                                    .andThen(
                                        new InstantCommand(
                                            () -> {
                                              driveSubsystem.setParkMode(true);
                                            })))),
                new InstantCommand(),
                () -> climb),
            new InstantCommand(() -> driveSubsystem.setHeadingOffest(180.0)))
        .withName(PATH_NAME);
  }

  public static Command getDynamicPathToCubeNode(DriveSubsystem driveSubsystem) {
    Pose2d cubeNodePose =
        LocationHelper.transformYAxisForAllianceColor(
            new Pose2d(2.3, 4.42, Rotation2d.fromDegrees(180.0)));
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
}
