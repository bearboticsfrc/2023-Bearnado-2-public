package frc.constants;

public final class ManipulatorConstants {
  public static final double NOMINAL_VOLTAGE = 12.0;
  public static final int CURRENT_LIMIT = 40;
  public static final boolean MOTOR_INVERTED = false;

  public static final int ELEVATOR_MOTOR_PORT = 42;
  public static final int ELEVATOR_MOTOR_FOLLOWER_PORT = 43;
  public static final boolean ELEVATOR_MOTOR_REVERSED = false;
  public static final int ELEVATOR_LIMIT_SWITCH_PORT = 9;
  public static final int DRAWER_MOTOR_PORT = 41;
  public static final boolean DRAWER_MOTOR_REVERSED = true;
  public static final int DRAWER_LIMIT_SWITCH_PORT = 8;
  public static final int WRIST_MOTOR_PORT = 40;
  public static final boolean WRIST_MOTOR_REVERSED = false;

  public static final int ROLLER_MOTOR_PORT = 30;
  public static final int ROLLER_SENSOR_ANALOG_PORT = 3;

  public static final double ELEVATOR_PID_UP_P = 0.05;
  public static final double ELEVATOR_PID_UP_I = 0.00006;
  public static final double ELEVATOR_PID_UP_D = .005;
  public static final double ELEVATOR_PID_UP_FF = 0;
  public static final double ELEVATOR_PID_UP_I_ZONE = 5.0;

  public static final double ELEVATOR_PID_DOWN_P = .035;
  public static final double ELEVATOR_PID_DOWN_I = 0.000001;
  public static final double ELEVATOR_PID_DOWN_D = 0.0;
  public static final double ELEVATOR_PID_DOWN_FF = 0;
  public static final double ELEVATOR_PID_DOWN_I_ZONE = 2.0;

  public static final double DRAWER_PID_P = 0.020;
  public static final double DRAWER_PID_I = 0;
  public static final double DRAWER_PID_D = 0.05;
  public static final double DRAWER_PID_FF = 0;
  public static final double DRAWER_PID_I_ZONE = 0;

  public static final double WRIST_PID_P = 1.6; // 1.8;
  public static final double WRIST_PID_I = 0;
  public static final double WRIST_PID_D = 0.6;
  public static final double WRIST_PID_FF = 0;
  public static final double WRIST_PID_I_ZONE = 0;

  public static final double ELEVATOR_MIN = 1.0;
  public static final double ELEVATOR_MAX = 34.5; // 66.0, 36.6666
  public static final double ELEVATOR_SHELF = 32.222; // 58.0,32.2222
  public static final double ELEVATOR_MID = 27.777; // 50.0, 27.777
  public static final double ELEVATOR_MID_CONE = 29.444; // 53.0, 29.4444
  public static final double ELEVATOR_ALMOST_UP = 25.0; // 45.0, 25.0
  public static final double ELEVATOR_SAFE_MAX = 5.555; // 10.0, 5.5555
  public static final double ELEVATOR_BUFFER = 4.444; // 8.0, 4.4444
  public static final double ELEVATOR_HOME = 1.0;

  public static final double DRAWER_MIN = 1.0;
  public static final double DRAWER_MAX = 97.0; // 97
  public static final double DRAWER_HIGH_CUBE = 85.0;
  public static final double DRAWER_SHELF = 1.0;
  public static final double CONE_DRAWER_MID = 55.0;
  public static final double CUBE_DRAWER_MID = 30.0;
  public static final double DRAWER_HOME = 1.0;

  // wrist minimum value is closer to the robot
  // higher values rotate out closer to the floor
  public static final double WRIST_MIN = 0.10;
  public static final double WRIST_MAX = .53;
  public static final double WRIST_HOME = 0.12;
  public static final double WRIST_SAFE_MIN = .17;
  public static final double WRIST_SAFE_MIN_HOME = .22;
  public static final double WRIST_SAFE_MAX = .50;
  public static final double WRIST_LOWER = .50;
  public static final double WRIST_STRAIGHT = .44;
  public static final double WRIST_SHELF = .44;
  public static final double WRIST_SLIGHT_UP = .43;
  public static final double WRIST_FOR_MID_POS = .43;
  public static final double WRIST_TOP_SCORE_POSITION = 0.38;
  public static final double WRIST_30_UP = .38;
  public static final double WRIST_SHOOT = .30;

  // speed range -0.8 - 0.8

  public static final double RAISE_ELEVATOR_SPEED = 0.5;
  public static final double LOWER_ELEVATOR_SPEED = -0.2;

  public static final double EXTEND_DRAWER_SPEED = 0.2;
  public static final double RETRACT_DRAWER_SPEED = -0.2;

  public static final double FLICK_WRIST_UP_SPEED = 0.2;
  public static final double FLICK_WRIST_DOWN_SPEED = -0.2;

  public static final double ROLLER_DROP_SPEED = -1.0;
  public static final double ROLLER_GRAB_SPEED = 0.75;
}
