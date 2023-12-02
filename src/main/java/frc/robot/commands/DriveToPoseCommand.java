package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.vision.StringFormatting;

public class DriveToPoseCommand {

  private final DriveSubsystem driveSubsystem;
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;

  PathPlannerTrajectory trajectory = null;

  public DriveToPoseCommand(
      DriveSubsystem driveSubsystem, PoseEstimatorSubsystem poseEstimatorSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
  }

  public Command get(Pose2d targetPose) {
    return new InstantCommand(
            () -> {
              Translation2d currentSpeed =
                  LocationHelper.getFieldRelativeLinearSpeedsMPS(
                      driveSubsystem, poseEstimatorSubsystem);
              DataLogManager.log("Driving to " + StringFormatting.poseToString(targetPose));
              trajectory =
                  LocationHelper.generateTrajectory(
                      poseEstimatorSubsystem.getPose(), targetPose, currentSpeed);
            })
        .andThen(
            new ProxyCommand(
                () ->
                    LocationHelper.followTrajectoryCommand(
                        trajectory, false, driveSubsystem, poseEstimatorSubsystem)));
  }
}
