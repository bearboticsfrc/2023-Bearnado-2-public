package frc.robot.commands.auto;

import static frc.constants.AutoConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.PathCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubystem;

public class RedCubeLeftCS {

  public static Command get(DriveSubsystem driveSubsystem, ManipulatorSubystem manipulator) {

    final String PATH_NAME = RED_CUBE_LEFT_CS_PATH_NAME;

    PathPlannerTrajectory pathPlannerTrajectory =
        PathPlanner.loadPath(
            PATH_NAME, MAX_SPEED_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    Command pathCommand = new PathCommand(driveSubsystem, pathPlannerTrajectory);

    return new SequentialCommandGroup(
            new InstantCommand(
                () -> driveSubsystem.resetOdometry(pathPlannerTrajectory.getInitialState())),
            manipulator.getShootCubeCommand(),
            new InstantCommand(() -> manipulator.release()),
            pathCommand,
            new AutoBalanceCommand(driveSubsystem)
                .alongWith(
                    new WaitUntilCommand(14.0)
                        .andThen(new InstantCommand(() -> driveSubsystem.setParkMode(true)))))
        .withName(PATH_NAME);
  }
}
