package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class CircleCommand extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final double SCALE = .15;
  private final double STEP = 0.0174533;
  private double angleRadians = 0.0;

  public CircleCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    angleRadians += STEP;

    if (angleRadians > (2.0 * Math.PI)) {
      angleRadians = 0;
    }

    double x = SCALE * Math.cos(angleRadians);
    double y = SCALE * Math.sin(angleRadians);

    driveSubsystem.drive(x, y, 0, false);
  }
}
