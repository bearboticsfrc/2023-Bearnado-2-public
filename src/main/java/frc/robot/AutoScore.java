package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.location.FieldPositions;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import java.util.function.LongSupplier;

public class AutoScore {

  // value from [1..9]
  int targetGridColumn = 9;

  enum GridRow {
    TOP,
    MIDDLE,
    BOTTOM
  }

  GridRow targetGridRow = GridRow.BOTTOM;

  ShuffleboardTab tab = Shuffleboard.getTab("Auto Score");
  private SendableChooser<Integer> columnChooser = new SendableChooser<>();
  private SendableChooser<GridRow> rowChooser = new SendableChooser<>();

  private DriveSubsystem driveSubsystem;
  private PoseEstimatorSubsystem poseSubsystem;
  private LongSupplier columnSource;
  private FieldPositions fieldPositions;

  public AutoScore(
      DriveSubsystem driveSubsystem,
      PoseEstimatorSubsystem poseSubsystem,
      LongSupplier columnSource,
      FieldPositions fieldPositions) {
    this.columnSource = columnSource;
    this.driveSubsystem = driveSubsystem;
    this.poseSubsystem = poseSubsystem;
    this.fieldPositions = fieldPositions;
    buildShuffleboardWidget();
  }

  public Integer getSelectedColumn() {
    return columnChooser.getSelected();
  }

  public void buildShuffleboardWidget() {
    tab.add("Target Column", columnChooser).withPosition(1, 1);

    for (int i = 1; i <= 9; i++) {
      columnChooser.addOption("" + i, Integer.valueOf(i));
    }
    columnChooser.setDefaultOption("9", Integer.valueOf(9));

    tab.add("Target Row", rowChooser).withPosition(2, 1);

    for (GridRow row : GridRow.values()) {
      rowChooser.addOption(row.toString(), row);
    }

    rowChooser.setDefaultOption(GridRow.TOP.toString(), GridRow.TOP);

    tab.addInteger("Column", columnSource);
  }

  PathPlannerTrajectory trajectory = null;

  public Command getDriveToGridCommand() {
    return new InstantCommand(
            () -> {
              DataLogManager.log("Starting DrivetoGrid");
              // Integer selected = getSelectedColumn();
              long column = columnSource.getAsLong();
              DataLogManager.log("Selected column: " + column);
              Integer intColumn = Integer.valueOf((int) column);
              Pose2d targetPose = fieldPositions.getGridPoseMap().get(intColumn);
              Translation2d currentSpeed =
                  LocationHelper.getFieldRelativeLinearSpeedsMPS(driveSubsystem, poseSubsystem);
              Pose2d currentPose = poseSubsystem.getPose();
              DataLogManager.log(
                  "*************** Driving from "
                      + currentPose.getX()
                      + ", "
                      + currentPose.getY()
                      + ", "
                      + currentPose.getRotation().getDegrees());
              DataLogManager.log(
                  "*************** Driving to "
                      + targetPose.getX()
                      + ", "
                      + targetPose.getY()
                      + ", "
                      + targetPose.getRotation().getDegrees());
              DataLogManager.log("current speed = " + currentSpeed.getNorm());
              trajectory =
                  LocationHelper.generateTrajectory(
                      poseSubsystem.getPose(), targetPose, currentSpeed);
            })
        .andThen(
            new ProxyCommand(
                () ->
                    LocationHelper.followTrajectoryCommand(
                        trajectory, false, driveSubsystem, poseSubsystem)));
  }
}
