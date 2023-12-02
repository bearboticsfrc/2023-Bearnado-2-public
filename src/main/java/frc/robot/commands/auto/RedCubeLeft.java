package frc.robot.commands.auto;

import static frc.constants.AutoConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.constants.ManipulatorConstants;
import frc.robot.commands.CubeHuntCommand;
import frc.robot.commands.LogCommand;
import frc.robot.commands.PathCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubystem;
import java.util.HashMap;

public class RedCubeLeft {

  public static Command get(DriveSubsystem driveSubsystem, ManipulatorSubystem manipulator) {

    final String PATH_NAME = RED_CUBE_LEFT_PATH_NAME;
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
            // Shoot the cube
            manipulator.getShootCubeCommand(),
            // new InstantCommand(() -> manipulator.grab()),
            // new InstantCommand(() -> manipulator.rollerDrop()),
            // new LogCommand("Started Rollers!!!!!!!!!!!!!!!!!!!!!!!!!!!!"),
            // new WaitCommand(2.0),
            // new InstantCommand(() -> manipulator.rollerStop()),
            // new LogCommand("Stopped Rollers!!!!!!!!!!!!!!!!!!!!!!!!!!!!"),

            command, // 2.36
            new CubeHuntCommand(driveSubsystem, () -> manipulator.getRollerSensor())
                .withTimeout(0.5),
            new InstantCommand(manipulator::rollerStop),
            new InstantCommand(manipulator::grab),
            new InstantCommand(driveSubsystem::stop),
            new InstantCommand(() -> manipulator.setWristReference(ManipulatorConstants.WRIST_MIN)),
            new LogCommand("Finished with " + PATH_NAME))
        .withName(PATH_NAME);
  }
}
