package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class AlignGridCommand extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final String LIMELIGHT_NAME = "limelight";
  private final PIDController ySpeedController = new PIDController(0.05, 0, 0);
  private final PIDController xSpeedController = new PIDController(0.05, 0, 0);

  public AlignGridCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);

    ySpeedController.setTolerance(0.1);
    xSpeedController.setTolerance(0.1);
  }

  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, 1);
  }

  @Override
  public void execute() {
    if (!LimelightHelpers.getTV(LIMELIGHT_NAME)) {
      driveSubsystem.drive(0, 0, 0);
      return;
    }

    double targetX = LimelightHelpers.getTX(LIMELIGHT_NAME);
    double targetY = LimelightHelpers.getTY(LIMELIGHT_NAME);

    double ySpeed = ySpeedController.calculate(targetX, 0);
    double xSpeed = xSpeedController.calculate(targetY, 0);

    driveSubsystem.drive(-xSpeed, ySpeed, 0, false);
  }

  @Override
  public boolean isFinished() {
    return ySpeedController.atSetpoint() && xSpeedController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    // LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, 0);
  }
}
