package frc.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class AutoConstants {
  public static final double MAX_SPEED_METERS_PER_SECOND = 2;
  public static final double POWER_STRIP_MAX_METERS_PER_SECOND = 1;

  public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2;
  public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;

  public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

  public static final double PX_CONTROLLER = 2;
  public static final double PY_CONTROLLER = 2;
  public static final double PTHETA_CONTROLLER = 4;

  public static final String CHARGE_STATION_PATH_NAME = "ChargeStation";
  public static final String CSFROM_COMMUNITY_PATH_NAME = "CSFromCommunity";
  public static final String UNO_PIECE_ROJO_PATH_NAME = "UnoPieceRojo";
  public static final String UNO_PIECE_AZUL_PATH_NAME = "UnoPieceAzul";
  public static final String UPPER_UNO_PIECE_ROJO_PATH_NAME = "UpperUnoPieceRojo";
  public static final String UPPER_UNO_PIECE_AZUL_PATH_NAME = "UpperUnoPieceAzul";
  public static final String ANY_CUBE_ONLY = "[ANY]CubeOnly";
  public static final String MID_CONE_SCORE = "[RED]Cone";

  public static final String OVER_POWER_STRIP_PATH_NAME = "OverPowerStrip";
  public static final String POWER_STRIP_TO_CONE_PATH_NAME = "PowerStripToCone";

  //// the real autos .....
  public static final String LEAVE_COMMUNITY_PATH_NAME = "LeaveCommunity";

  public static final String BLUE_CUBE_LEFT_PATH_NAME = "[BLUE]CubeLeft";
  public static final String RED_CUBE_LEFT_PATH_NAME = "[RED]CubeLeft";
  public static final String BLUE_CUBE_RIGHT_PATH_NAME = "[BLUE]CubeRight";
  public static final String RED_CUBE_RIGHT_PATH_NAME = "[RED]CubeRight";

  public static final String RED_CUBE_RIGHT_CS_PATH_NAME = "[RED]CubeRightCS";
  public static final String RED_CUBE_LEFT_CS_PATH_NAME = "[RED]CubeLeftCS";

  public static final String BLUE_CUBE_LEFT_CS_PATH_NAME = "[BLUE]CubeLeftCS";
  public static final String BLUE_CUBE_RIGHT_CS_PATH_NAME = "[BLUE]CubeRightCS";

  public static final String RED_CONE_RIGHT = "[RED]ConeRight";
  public static final String BLUE_CONE_SUBSTATION = "[BLUE]Cone-Substation";

  public static final String MIDDLE_CUBE_ENGAGE = "MiddleCubeEngage";
  public static final String MIDDLE_CUBE_MOBILITY_ENGAGE = "MiddleCubeMobilityEngage";

  // Constraint for the motion profilied robot angle controller
  // public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new
  // TrapezoidProfile.Constraints(
  //    kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

  public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
          DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
              * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

  // P value used in the AutoRotate command
}
