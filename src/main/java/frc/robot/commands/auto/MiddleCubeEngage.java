package frc.robot.commands.auto;

import static frc.constants.AutoConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.PathCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubystem;

public class MiddleCubeEngage {

  public static Command get(DriveSubsystem driveSubsystem, ManipulatorSubystem manipulator) {

    final String PATH_NAME = MIDDLE_CUBE_ENGAGE;
    PathPlannerTrajectory pathPlannerTrajectory =
        PathPlanner.loadPath(PATH_NAME, 1.9, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(
            manipulator.getShootCubeCommand(),
            new ProxyCommand(
                () -> new PathCommand(driveSubsystem, pathPlannerTrajectory, false, true)),
            new AutoBalanceCommand(driveSubsystem)
                .alongWith(
                    new WaitUntilCommand(14.9)
                        .andThen(new InstantCommand(() -> driveSubsystem.setParkMode(true)))),
            // new LogCommand("After follow path command, about to run cube hunt."),
            // new CubeHuntCommand(driveSubsystem, () ->
            // manipulator.getRollerSensor()).withTimeout(2.0),
            // new InstantCommand(() -> manipulator.rollerStop()),
            // new InstantCommand(() -> manipulator.grab()),
            new InstantCommand(() -> driveSubsystem.stop()))
        .withName(PATH_NAME);
  }
}
