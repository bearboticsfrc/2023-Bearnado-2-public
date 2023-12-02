package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubystem;

public class DriveToLoadingCommand extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final ManipulatorSubystem manipulator;

  private final PIDController xSpeedController = new PIDController(0.05, 0, 0.001);

  public DriveToLoadingCommand(DriveSubsystem driveSubsystem, ManipulatorSubystem manipulator) {
    this.driveSubsystem = driveSubsystem;
    this.manipulator = manipulator;
    xSpeedController.setSetpoint(0.5);
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {

    double xSpeed = xSpeedController.calculate(manipulator.getRollerSensor());

    driveSubsystem.drive(xSpeed, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    return xSpeedController.atSetpoint();
  }
}
