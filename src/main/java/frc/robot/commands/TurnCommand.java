package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class TurnCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final PIDController rotSpeedController = new PIDController(0.01, 0, 0.001);
  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;

  public TurnCommand(
      DriveSubsystem driveSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      double angleDegrees) {
    this.driveSubsystem = driveSubsystem;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    rotSpeedController.setSetpoint(angleDegrees);
    rotSpeedController.enableContinuousInput(0, 360.0);
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    double rot = rotSpeedController.calculate(driveSubsystem.getHeading().getDegrees());

    driveSubsystem.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), rot, true);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(rotSpeedController.getSetpoint() - driveSubsystem.getHeading().getDegrees())
        < 0.5;
  }
}
