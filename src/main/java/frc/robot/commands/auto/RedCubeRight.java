package frc.robot.commands.auto;

import static frc.constants.AutoConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CubeHuntCommand;
import frc.robot.commands.PathCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubystem;
import java.util.HashMap;

public class RedCubeRight {

  public static Command get(DriveSubsystem driveSubsystem, ManipulatorSubystem manipulator) {

    final String PATH_NAME = RED_CUBE_RIGHT_PATH_NAME;

    PathPlannerTrajectory pathPlannerTrajectory =
        PathPlanner.loadPath(
            PATH_NAME, MAX_SPEED_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    Command pathCommand = new PathCommand(driveSubsystem, pathPlannerTrajectory);

    HashMap<String, Command> eventMap = manipulator.getEventMap();

    FollowPathWithEvents command =
        new FollowPathWithEvents(pathCommand, pathPlannerTrajectory.getMarkers(), eventMap);

    return new SequentialCommandGroup(
            new InstantCommand(
                () -> driveSubsystem.resetOdometry(pathPlannerTrajectory.getInitialState())),
            manipulator.getShootCubeCommand(),
            command,
            new CubeHuntCommand(driveSubsystem, () -> manipulator.getRollerSensor())
                .andThen(manipulator.getIntakeObjectCommand())
                .andThen(new InstantCommand(() -> manipulator.grab())),
            manipulator.getHomeCommand(),
            /*  new PathCommand(
                driveSubsystem,
                TrajectoryGenerator.generateTrajectory(
                    driveSubsystem.getPose(),
                    new Pose2d(14.67, 4.42, Rotation2d.fromDegrees(-180.0)))),
            manipulator.getShootCubeCommand(), */
            new InstantCommand(() -> driveSubsystem.stop()))
        .withName(PATH_NAME);
  }
}
