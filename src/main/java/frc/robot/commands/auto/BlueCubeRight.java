package frc.robot.commands.auto;

import static frc.constants.AutoConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CubeHuntCommand;
import frc.robot.commands.LogCommand;
import frc.robot.commands.PathCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubystem;

public class BlueCubeRight {

  public static Command get(DriveSubsystem driveSubsystem, ManipulatorSubystem manipulator) {

    final String PATH_NAME = BLUE_CUBE_RIGHT_PATH_NAME;
    PathPlannerTrajectory pathPlannerTrajectory =
        PathPlanner.loadPath(
            PATH_NAME, MAX_SPEED_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(
            manipulator.getShootCubeCommand(),
            new ProxyCommand(
                () ->
                    new FollowPathWithEvents(
                        new PathCommand(driveSubsystem, pathPlannerTrajectory, false, true),
                        pathPlannerTrajectory.getMarkers(),
                        manipulator.getEventMap())),
            new LogCommand("After follow path command, about to run cube hunt."),
            new CubeHuntCommand(driveSubsystem, () -> manipulator.getRollerSensor())
                .withTimeout(1.0),
            new InstantCommand(() -> manipulator.rollerStop()),
            new InstantCommand(() -> manipulator.grab()),
            new InstantCommand(() -> driveSubsystem.stop()))
        .withName(PATH_NAME);
  }
}
