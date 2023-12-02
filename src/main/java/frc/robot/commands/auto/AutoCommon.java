package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.LogCommand;
import frc.robot.commands.PathCommand;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;

public class AutoCommon {

  /*
   * Make a dynamic path to center of community near the grid, then drive up the charge station.   Assumes robot is on the substation side.
   *
   */
  public static Command getDynamicPathToClimb(DriveSubsystem driveSubsystem) {
    Pose2d chargeStationPose =
        LocationHelper.transformYAxisForAllianceColor(
            new Pose2d(5.5, 2.92, Rotation2d.fromDegrees(180.0)));
    Pose2d entryPose =
        LocationHelper.transformYAxisForAllianceColor(
            new Pose2d(2.3, 2.92, Rotation2d.fromDegrees(180.0)));

    PathPlannerTrajectory blueTrajectory =
        PathPlanner.generatePath(
            new PathConstraints(2.25, 4),
            new PathPoint(
                    driveSubsystem.getPose().getTranslation(),
                    LocationHelper.transformHeadingForAllianceColor(Rotation2d.fromDegrees(-90.0)),
                    driveSubsystem.getPose().getRotation())
                .withNextControlLength(1.0),
            new PathPoint(
                    entryPose.getTranslation(),
                    entryPose.getRotation(),
                    Rotation2d.fromDegrees(0.0))
                .withControlLengths(0.01, 0.01),
            new PathPoint(
                    chargeStationPose.getTranslation(),
                    chargeStationPose.getRotation(),
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
