package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.location.FieldPositions;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.vision.StringFormatting;
import java.util.List;

public class DriveToNearestPoseCommand {

  private final DriveSubsystem driveSubsystem;
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;
  private final FieldPositions fieldPositions;

  PathPlannerTrajectory trajectory = null;

  public DriveToNearestPoseCommand(
      DriveSubsystem driveSubsystem,
      PoseEstimatorSubsystem poseEstimatorSubsystem,
      FieldPositions fieldPositions) {
    this.driveSubsystem = driveSubsystem;
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    this.fieldPositions = fieldPositions;
  }

  public Command get() {
    return get(fieldPositions.getPoseList());
  }

  public Command get(List<Pose2d> poseList) {
    Translation2d currentSpeed =
        LocationHelper.getFieldRelativeLinearSpeedsMPS(driveSubsystem, poseEstimatorSubsystem);
    return new InstantCommand(
            () -> {
              Pose2d currentPose = poseEstimatorSubsystem.getPose();
              Pose2d targetPose = currentPose.nearest(poseList);
              DataLogManager.log("Driving to " + StringFormatting.poseToString(targetPose));
              trajectory = LocationHelper.generateTrajectory(currentPose, targetPose, currentSpeed);
            })
        .andThen(
            new ProxyCommand(
                () ->
                    LocationHelper.followTrajectoryCommand(
                        trajectory, false, driveSubsystem, poseEstimatorSubsystem)));
  }
}
