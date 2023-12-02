package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class PIDToPoseCommand extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final Pose2d targetPose;
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;

  private final ProfiledPIDController xPIDController = new ProfiledPIDController(.1, 0, 0, null);
  private final ProfiledPIDController yPIDController = new ProfiledPIDController(.1, 0, 0, null);
  private final ProfiledPIDController thetaPIDController =
      new ProfiledPIDController(.1, 0, 0, null);

  public PIDToPoseCommand(
      DriveSubsystem driveSubsystem,
      Pose2d targetPose,
      PoseEstimatorSubsystem poseEstimatorSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.targetPose = targetPose;
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    Transform2d transform = new Transform2d(poseEstimatorSubsystem.getPose(), targetPose);

    double x = xPIDController.calculate(transform.getX());
    double y = yPIDController.calculate(transform.getY());
    double theta = thetaPIDController.calculate(transform.getRotation().getDegrees());

    driveSubsystem.drive(x, y, theta, false);
  }

  @Override
  public boolean isFinished() {
    return xPIDController.atGoal() && yPIDController.atGoal() && thetaPIDController.atGoal();
  }
}
