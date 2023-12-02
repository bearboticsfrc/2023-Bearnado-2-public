// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.DriveConstants;
import frc.robot.util.CTREUtil;
import frc.robot.util.RateLimiter;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Map;

/** Controls the four swerve modules for autonomous and teleoperated modes. */
public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule frontLeft;
  private final SwerveModule backLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backRight;

  private final Path ODOMETER_PATH = Paths.get("/home/lvuser/deploy/odometer.txt");

  // The pigeon2 sensor
  public final WPI_Pigeon2 pigeon2;

  private double maxSpeed = DriveConstants.MAX_SPEED_METERS_PER_SECOND / 2;
  private double previousSpeed = maxSpeed;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry;

  ShuffleboardTab driveSystemTab = Shuffleboard.getTab("Drive System");
  ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

  GenericEntry maxSpeedEntry =
      compTab
          .add("Drive Speed", maxSpeed)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withSize(2, 1)
          .withPosition(5, 1)
          .withProperties(Map.of("min", 0, "max", DriveConstants.MAX_SPEED_METERS_PER_SECOND))
          .getEntry();

  private boolean fieldRelativeMode = true;

  private final RateLimiter xLimiter =
      new RateLimiter(
          DriveConstants.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND,
          DriveConstants.TELE_DRIVE_MAX_DECELERATION_UNITS_PER_SECOND);
  private final RateLimiter yLimiter =
      new RateLimiter(
          DriveConstants.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND,
          DriveConstants.TELE_DRIVE_MAX_DECELERATION_UNITS_PER_SECOND);
  private final RateLimiter turningLimiter =
      new RateLimiter(
          DriveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND,
          DriveConstants.TELE_DRIVE_MAX_ANGULAR_DECELERATION_UNITS_PER_SECOND);

  private Cancoders cancoders;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    pigeon2 = new WPI_Pigeon2(DriveConstants.PIGEON2_CANID);
    CTREUtil.checkCtreError(
        pigeon2.configFactoryDefault(), "Failed to config factory default on pigeon");

    driveSystemTab.addDouble("Pitch", this::getPitch);
    driveSystemTab.addDouble("Roll", this::getRoll);
    driveSystemTab.addBoolean("Field Relative?", () -> fieldRelativeMode);
    compTab.addNumber("pigeon2 heading", () -> getHeading().getDegrees());

    zeroHeading();

    cancoders = Cancoders.getInstance();
    double startInitTs = Timer.getFPGATimestamp();
    System.out.println("* Starting to init cancoders at ts " + startInitTs);
    while (Timer.getFPGATimestamp() - startInitTs < 10.0 && !cancoders.allHaveBeenInitialized()) {
      Timer.delay(0.1);
    }
    System.out.println(
        "* Cancoders all inited: Took " + (Timer.getFPGATimestamp() - startInitTs) + " seconds");

    frontLeft = new SwerveModule(DriveConstants.SwerveModule.FRONT_LEFT, driveSystemTab);
    backLeft = new SwerveModule(DriveConstants.SwerveModule.BACK_LEFT, driveSystemTab);
    frontRight = new SwerveModule(DriveConstants.SwerveModule.FRONT_RIGHT, driveSystemTab);
    backRight = new SwerveModule(DriveConstants.SwerveModule.BACK_RIGHT, driveSystemTab);

    odometry =
        new SwerveDriveOdometry(
            DriveConstants.DRIVE_KINEMATICS, getHeading(), getModulePositions());
  }

  public double getPitch() {
    return pigeon2.getPitch();
  }

  public double getRoll() {
    return pigeon2.getRoll();
  }

  @Override
  public void periodic() {
    odometry.update(getHeading(), getModulePositions());

    maxSpeed = maxSpeedEntry.getDouble(DriveConstants.MAX_SPEED_METERS_PER_SECOND);

    frontLeft.updateDataLogs();
    backLeft.updateDataLogs();
    frontRight.updateDataLogs();
    backRight.updateDataLogs();
  }

  public void restorePreviousSpeed() {
    maxSpeed = previousSpeed;
  }

  public void setMaxSpeed() {
    previousSpeed = maxSpeed;
    maxSpeed = DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    maxSpeedEntry.setDouble(maxSpeed);
  }

  public void setTurtleMode(boolean mode) {
    maxSpeed = mode ? 0.5 : DriveConstants.MAX_SPEED_METERS_PER_SECOND / 2;

    maxSpeedEntry.setDouble(maxSpeed);
  }

  /**
   * @param turboMode
   */
  public void setTurboMode(boolean mode) {
    // speed = mode ? Math.min(speed * 2.0, DriveConstants.MAX_SPEED_METERS_PER_SECOND) : speed / 2;
    maxSpeed = mode ? DriveConstants.MAX_SPEED_METERS_PER_SECOND : maxSpeed / 2.0;

    maxSpeedEntry.setDouble(maxSpeed);
  }

  /**
   * Sets the robot to be field relative or not.
   *
   * @param mode The mode.
   */
  public void setFieldRelative(boolean mode) {
    fieldRelativeMode = mode;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getHeading(), getModulePositions(), pose);
    // pigeon2.addYaw(pose.getRotation().getDegrees());
  }

  /**
   * Returns the state of every swerve module.
   *
   * @return The states.
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()
    };
  }

  /**
   * Resets the odometry to the specified pose of a state in a PathPlanner trajectory.
   *
   * @param state The state of the PathPlanner trajectory to contstruct a pose.
   */
  public void resetOdometry(PathPlannerState state) {
    resetOdometry(new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation));
  }

  /**
   * Drive robot using Joystick inputs, default to CENTER pivot, and sets field relative to the
   * current fieldRelativeMode setting.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   */
  public void drive(double xSpeed, double ySpeed, double rot) {
    this.drive(xSpeed, ySpeed, rot, fieldRelativeMode);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    xSpeed = xLimiter.calculate(xSpeed) * maxSpeed; // DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    ySpeed = yLimiter.calculate(ySpeed) * maxSpeed; // DriveConstants.MAX_SPEED_METERS_PER_SECOND;

    if (maxSpeed == 0.5) {
      rot =
          turningLimiter.calculate(rot)
              * DriveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND
              / 4.0;
    } else {
      rot =
          turningLimiter.calculate(rot)
              * DriveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
    }
    var swerveModuleStates =
        DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));

    setModuleStates(swerveModuleStates);
  }

  /** Turn off the drive motors */
  public void stop() {
    drive(0.0, 0.0, 0.0);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param swerveModuleStates The desired swerve module states.
   */
  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);

    frontLeft.set(swerveModuleStates[0]);
    frontRight.set(swerveModuleStates[1]);
    backLeft.set(swerveModuleStates[2]);
    backRight.set(swerveModuleStates[3]);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    pigeon2.reset();
    //  if (odometry != null) {
    //    resetOdometry(getPose());
    //  }
  }

  public void setHeadingOffest(double offset) {
    System.out.println("#################  Adding offset " + offset);
    pigeon2.addYaw(offset);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading as a Rotation2d
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(
        MathUtil.inputModulus(pigeon2.getRotation2d().getDegrees(), 0, 360));
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second.
   */
  public double getTurnRate() {
    return pigeon2.getRate();
  }

  /**
   * Returns the position of every swerve module.
   *
   * @return The positions.
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition(),
    };
  }

  public void setParkMode(boolean enabled) {
    for (SwerveModule module : new SwerveModule[] {frontLeft, frontRight, backLeft, backRight}) {
      if (!enabled) {
        module.setParkMode(false);
        continue;
      }

      SwerveModuleState state = module.getState();

      state.speedMetersPerSecond = 0;
      state.angle = module.getParkedDegrees();

      module.set(state);
      module.setParkMode(true);
    }
  }
}
