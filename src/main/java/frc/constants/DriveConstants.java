package frc.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class DriveConstants extends DriveConstants2023 {

  // Max drive velocity in meters per second
  public static final double MAX_VELOCITY_METERS_PER_SECOND =
      MAX_MOTOR_FREE_SPEED_RPM / 60.0 * DRIVE_GEAR_REDUCTION * WHEEL_DIAMETER_METERS * Math.PI;

  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
      MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(SELF_TRACK_WIDTH / 2.0, SELF_WHEEL_BASE / 2.0);

  public static final double PARK_MODE_ANGLE_RIGHT = 135.0;
  public static final double PARK_MODE_ANGLE_LEFT = 225.0;

  public static final SwerveDriveKinematics DRIVE_KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(SELF_WHEEL_BASE / 2, SELF_TRACK_WIDTH / 2), // front left
          new Translation2d(SELF_WHEEL_BASE / 2, -SELF_TRACK_WIDTH / 2), // front right
          new Translation2d(-SELF_WHEEL_BASE / 2, SELF_TRACK_WIDTH / 2), // back left
          new Translation2d(-SELF_WHEEL_BASE / 2, -SELF_TRACK_WIDTH / 2)); // back right

  public static final double MAX_SPEED_METERS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND;
  public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND =
      MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
  public static final double TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND =
      MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

  public enum SwerveModule {
    // here you go kellan this is clean code
    FRONT_LEFT(
        "FL",
        FRONT_LEFT_DRIVE_MOTOR_PORT,
        FRONT_LEFT_PIVOT_CANCODER_PORT,
        FRONT_LEFT_PIVOT_MOTOR_PORT,
        FRONT_LEFT_ENCODER_ANGLE,
        Rotation2d.fromDegrees(PARK_MODE_ANGLE_LEFT),
        FRONT_LEFT_DRIVE_MOTOR_INVERTED,
        FRONT_LEFT_PIVOT_MOTOR_INVERTED),
    FRONT_RIGHT(
        "FR",
        FRONT_RIGHT_DRIVE_MOTOR_PORT,
        FRONT_RIGHT_PIVOT_INPUT_PORT,
        FRONT_RIGHT_PIVOT_MOTOR_PORT,
        FRONT_RIGHT_ENCODER_ANGLE,
        Rotation2d.fromDegrees(PARK_MODE_ANGLE_RIGHT),
        BACK_LEFT_DRIVE_MOTOR_INVERTED,
        BACK_LEFT_PIVOT_MOTOR_INVERTED),
    BACK_LEFT(
        "BL",
        BACK_LEFT_DRIVE_MOTOR_PORT,
        BACK_LEFT_PIVOT_INPUT_PORT,
        BACK_LEFT_PIVOT_MOTOR_PORT,
        BACK_LEFT_ENCODER_ANGLE,
        Rotation2d.fromDegrees(PARK_MODE_ANGLE_RIGHT),
        FRONT_RIGHT_DRIVE_MOTOR_INVERTED,
        FRONT_RIGHT_PIVOT_MOTOR_INVERTED),
    BACK_RIGHT(
        "BR",
        BACK_RIGHT_DRIVE_MOTOR_PORT,
        BACK_RIGHT_PIVOT_INPUT_PORT,
        BACK_RIGHT_PIVOT_MOTOR_PORT,
        BACK_RIGHT_ENCODER_ANGLE,
        Rotation2d.fromDegrees(PARK_MODE_ANGLE_LEFT),
        BACK_RIGHT_DRIVE_MOTOR_INVERTED,
        BACK_RIGHT_PIVOT_MOTOR_INVERTED);

    private final String moduleName;
    private final int driveMotorPort;
    private final int canCoderId;
    private final int pivotMotorPort;
    private final Rotation2d encoderOffsetDegrees;
    private final Rotation2d parkedDegrees;
    private final boolean driveMotorInverted;
    private final boolean pivotMotorInverted;

    SwerveModule(
        String moduleName,
        int driveMotorPort,
        int canCoderId,
        int pivotMotorPort,
        double encoderOffsetDegrees,
        Rotation2d parkedDegrees,
        boolean driveMotorInverted,
        boolean pivotMotorInverted) {
      this.moduleName = moduleName;
      this.driveMotorPort = driveMotorPort;
      this.canCoderId = canCoderId;
      this.pivotMotorPort = pivotMotorPort;
      this.encoderOffsetDegrees = Rotation2d.fromDegrees(encoderOffsetDegrees);
      this.parkedDegrees = parkedDegrees;
      this.driveMotorInverted = driveMotorInverted;
      this.pivotMotorInverted = pivotMotorInverted;
    }

    public String moduleName() {
      return moduleName;
    }

    public int driveMotorPort() {
      return driveMotorPort;
    }

    public int pivotMotorPort() {
      return pivotMotorPort;
    }

    public int canCoderId() {
      return canCoderId;
    }

    public Rotation2d encoderOffsetDegrees() {
      return encoderOffsetDegrees;
    }

    public Rotation2d parkedDegrees() {
      return parkedDegrees;
    }

    public boolean driveMotorInverted() {
      return driveMotorInverted;
    }

    public boolean pivotMotorInverted() {
      return pivotMotorInverted;
    }
  }

  public static final class ManipulatorConstants {
    public static final int ELEVATOR_MOTOR_PORT = 0;
    public static final int DRAWER_MOTOR_PORT = 0;
    public static final int WRIST_MOTOR_PORT = 0;
    public static final int ROLLER_MOTOR_PORT = 0;

    public static final double ELEVATOR_PID_P = 0;
    public static final double ELEVATOR_PID_I = 0;
    public static final double ELEVATOR_PID_D = 0;
    public static final double ELEVATOR_PID_I_ZONE = 0;

    public static final double DRAWER_PID_P = 0;
    public static final double DRAWER_PID_I = 0;
    public static final double DRAWER_PID_D = 0;
    public static final double DRAWER_PID_I_ZONE = 0;

    public static final double WRIST_PID_P = 0;
    public static final double WRIST_PID_I = 0;
    public static final double WRIST_PID_D = 0;
    public static final double WRIST_PID_I_ZONE = 0;

    public static final double ELEVATOR_MAX_HEIGHT = 0;
    public static final double DRAWER_MAX_OUT = 0;
    public static final double WRIST_MAX_UP = 0;

    public static final double ELEVATOR_MIN_HEIGHT = 0;
    public static final double DRAWER_MIN_OUT = 0;
    public static final double WRIST_MIN_UP = 0;

    // speed range -0.8 - 0.8

    public static final double RAISE_ELEVATOR_SPEED = 0.2;
    public static final double LOWER_ELEVATOR_SPEED = -0.2;

    public static final double EXTEND_DRAWER_SPEED = 0.2;
    public static final double RETRACT_DRAWER_SPEED = -0.2;

    public static final double FLICK_WRIST_UP_SPEED = 0.2;
    public static final double FLICK_WRIST_DOWN_SPEED = -0.2;

    public static final double ROLLER_GRAB_SPEED = 1;
    public static final double ROLLER_DROP_SPEED = -1;
  }
}
