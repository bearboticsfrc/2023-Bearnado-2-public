package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class CubeHuntCommand extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final DoubleSupplier sensor;
  private final String LIMELIGHT_NAME = "limelight";
  private final PIDController rotSpeedController = new PIDController(0.01, 0, 0.001);
  private final PIDController xSpeedController = new PIDController(0.1, 0, 0);

  public CubeHuntCommand(DriveSubsystem driveSubsystem, DoubleSupplier sensor) {
    this.driveSubsystem = driveSubsystem;
    this.sensor = sensor;

    addRequirements(driveSubsystem);
    rotSpeedController.setTolerance(2);
    xSpeedController.setTolerance(2);
  }

  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, 0);
  }

  @Override
  public void execute() {
    if (!LimelightHelpers.getTV(LIMELIGHT_NAME)) {
      driveSubsystem.drive(0, 0, 0);
      return;
    }

    double targetY = LimelightHelpers.getTY(LIMELIGHT_NAME);
    double targetX = LimelightHelpers.getTX(LIMELIGHT_NAME);

    double xSpeed = -xSpeedController.calculate(targetY, 0);
    double rot = rotSpeedController.calculate(targetX, 0);

    if (Math.abs(targetX) > 5) {
      // We want to prevent the manipulator
      // from accidently knocking the cube away.
      xSpeed = 0;
    }

    driveSubsystem.drive(xSpeed, 0, rot, false);
  }

  @Override
  public boolean isFinished() {
    return sensor.getAsDouble() < 1.2
        || (rotSpeedController.atSetpoint() && xSpeedController.atSetpoint());
  }
}
