package frc.constants;

public class DriveConstants2022 {

  // Track Width Distance between centers of right and left wheels on robot
  public static final double SELF_TRACK_WIDTH = 0.4445; // 0.61595;
  // Wheel Base Distance between front and back wheels on robot
  public static final double SELF_WHEEL_BASE = 0.7334; // 0.61595;

  // NEO motor free spin max from documentation
  public static final double MAX_MOTOR_FREE_SPEED_RPM = 5676.0;

  // Configurations for the SDS MK4i L2 module
  public static final double STEER_DRIVE_REDUCTION =
      (15.0 / 32.0) * (10.0 / 60.0); // (14.0 / 50.0) * (10.0 / 60.0);
  public static final double DRIVE_GEAR_REDUCTION =
      (14.0 / 50.0)
          * (28.0 / 16.0)
          * (15.0 / 60.0); // (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
  public static final double WHEEL_DIAMETER_METERS =
      0.1016; // 0.10033; // May be incorrect. Verify sometime!!

  public static final double MAX_VOLTAGE = 12.0;

  // value in amps to limit the neo motor with setSmartCurrentLimit
  public static final int DRIVE_CURRENT_LIMIT = 40;

  // value in volts to set the voltage compensation on the SPARK MAX via
  // enableVoltageCompensation
  public static final double NOMINAL_VOLTAGE = 12.0;

  // value in amps to limit the neo motor with setSmartCurrentLimit
  public static final int STEER_CURRENT_LIMIT = 20;

  protected static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 8;
  protected static final double FRONT_LEFT_ENCODER_ANGLE = -169.722;
  protected static final int FRONT_LEFT_PIVOT_CANCODER_PORT = 28;
  protected static final int FRONT_LEFT_PIVOT_MOTOR_PORT = 9;
  protected static final boolean FRONT_LEFT_DRIVE_MOTOR_INVERTED = true;
  protected static final boolean FRONT_LEFT_PIVOT_MOTOR_INVERTED = false;

  protected static final int BACK_LEFT_DRIVE_MOTOR_PORT = 4;
  protected static final int BACK_LEFT_PIVOT_MOTOR_PORT = 5;
  protected static final int BACK_LEFT_PIVOT_INPUT_PORT = 25;
  protected static final double BACK_LEFT_ENCODER_ANGLE = -24.53;
  protected static final boolean BACK_LEFT_DRIVE_MOTOR_INVERTED = true;
  protected static final boolean BACK_LEFT_PIVOT_MOTOR_INVERTED = false;

  protected static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 10;
  protected static final int FRONT_RIGHT_PIVOT_MOTOR_PORT = 11;
  protected static final int FRONT_RIGHT_PIVOT_INPUT_PORT = 26;
  protected static final double FRONT_RIGHT_ENCODER_ANGLE = -12.2;
  protected static final boolean FRONT_RIGHT_DRIVE_MOTOR_INVERTED = true;
  protected static final boolean FRONT_RIGHT_PIVOT_MOTOR_INVERTED = false;

  protected static final int BACK_RIGHT_DRIVE_MOTOR_PORT = 14;
  protected static final int BACK_RIGHT_PIVOT_MOTOR_PORT = 15;
  protected static final int BACK_RIGHT_PIVOT_INPUT_PORT = 27;
  protected static final double BACK_RIGHT_ENCODER_ANGLE = -65.1;
  protected static final boolean BACK_RIGHT_DRIVE_MOTOR_INVERTED = true;
  protected static final boolean BACK_RIGHT_PIVOT_MOTOR_INVERTED = false;

  public static int PIGEON2_CANID = 24;

  public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 4;
  public static final double TELE_DRIVE_MAX_DECELERATION_UNITS_PER_SECOND = 4;
  public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 10;
  public static final double TELE_DRIVE_MAX_ANGULAR_DECELERATION_UNITS_PER_SECOND = 20;
}
