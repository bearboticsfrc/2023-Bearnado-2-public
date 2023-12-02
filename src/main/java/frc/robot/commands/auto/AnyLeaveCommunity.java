package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.constants.AutoConstants;
import frc.robot.commands.PathCommand;
import frc.robot.subsystems.DriveSubsystem;

public class AnyLeaveCommunity {

  public static Command get(DriveSubsystem driveSubsystem) {

    PathPlannerTrajectory pathPlannerTrajectory =
        PathPlanner.loadPath(
            "BlueLeaveCommunity",
            // AutoConstants.LEAVE_COMMUNITY_PATH_NAME,
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    return new SequentialCommandGroup(
            new ProxyCommand(() -> new PathCommand(driveSubsystem, pathPlannerTrajectory)),
            new InstantCommand(() -> driveSubsystem.stop()))
        .withName(AutoConstants.LEAVE_COMMUNITY_PATH_NAME);
  }
}
