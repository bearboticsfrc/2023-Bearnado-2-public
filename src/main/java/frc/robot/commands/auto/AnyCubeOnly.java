package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubystem;

public class AnyCubeOnly {

  public static Command get(DriveSubsystem driveSubsystem, ManipulatorSubystem manipulator) {

    return new SequentialCommandGroup(manipulator.getShootCubeCommand())
        .withName(AutoConstants.ANY_CUBE_ONLY);
  }
}
