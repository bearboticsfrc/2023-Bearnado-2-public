package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.wpilibj.Timer;
import frc.constants.DriveConstants;
import frc.robot.util.CTREUtil;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

// Conatiner to hold the Cancoders so we can initialize them
// earlier than everything else and DI them to the swerve modules
public class Cancoders {
  private final CANCoder mFrontLeft;
  private final CANCoder mFrontRight;
  private final CANCoder mBackLeft;
  private final CANCoder mBackRight;

  private final CanTsObserver mFrontRightObserver;
  private final CanTsObserver mFrontLeftObserver;
  private final CanTsObserver mBackLeftObserver;
  private final CanTsObserver mBackRightObserver;

  private static final double kBootUpErrorAllowanceTime = 10.0;

  private Map<Integer, CANCoder> cancoders = new HashMap<Integer, CANCoder>();

  private static class CanTsObserver {
    private final CANCoder cancoder;
    private Optional<Double> lastTs = Optional.empty();
    private int validUpdates = 0;
    private static final int kRequiredValidTimestamps = 10;

    public CanTsObserver(CANCoder cancoder) {
      this.cancoder = cancoder;
    }

    public boolean hasUpdate() {
      cancoder.getAbsolutePosition(); // Need to call this to update ts
      double ts = cancoder.getLastTimestamp();
      if (lastTs.isEmpty()) {
        lastTs = Optional.of(ts);
      }
      if (ts > lastTs.get()) {
        validUpdates++;
        lastTs = Optional.of(ts);
      }
      return validUpdates > kRequiredValidTimestamps;
    }
  }

  private static Cancoders sInstance;

  public static Cancoders getInstance() {
    if (sInstance == null) {
      sInstance = new Cancoders();
    }
    return sInstance;
  }

  private CANCoder build(int canDeviceId, double offsetDegrees) {
    CANCoder thisCancoder = new CANCoder(canDeviceId);
    CANCoderConfiguration canCoderConfig = new CANCoderConfiguration();
    canCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    canCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    canCoderConfig.magnetOffsetDegrees = offsetDegrees;
    canCoderConfig.sensorDirection = false; // Counter-clockwise

    double startTime = Timer.getFPGATimestamp();
    boolean timedOut = false;
    boolean goodInit = false;
    int attempt = 1;
    while (!goodInit && !timedOut) {
      System.out.println("Initing CANCoder " + canDeviceId + " / attempt: " + attempt);
      ErrorCode settingsError = thisCancoder.configAllSettings(canCoderConfig, 100);
      ErrorCode sensorDataError =
          thisCancoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 50, 100);
      CTREUtil.checkCtreError(settingsError, "Failed to configure CANCoder");
      CTREUtil.checkCtreError(sensorDataError, "Failed to configure CANCoder update rate");

      goodInit = settingsError == ErrorCode.OK && sensorDataError == ErrorCode.OK;
      timedOut = (Timer.getFPGATimestamp()) - startTime >= kBootUpErrorAllowanceTime;
      attempt++;
    }

    return thisCancoder;
  }

  private Cancoders() {
    mFrontLeft =
        build(
            DriveConstants.SwerveModule.FRONT_LEFT.canCoderId(),
            DriveConstants.SwerveModule.FRONT_LEFT.encoderOffsetDegrees().getDegrees());
    mFrontLeftObserver = new CanTsObserver(mFrontLeft);
    cancoders.put(DriveConstants.SwerveModule.FRONT_LEFT.canCoderId(), mFrontLeft);

    mFrontRight =
        build(
            DriveConstants.SwerveModule.FRONT_RIGHT.canCoderId(),
            DriveConstants.SwerveModule.FRONT_RIGHT.encoderOffsetDegrees().getDegrees());
    mFrontRightObserver = new CanTsObserver(mFrontRight);
    cancoders.put(DriveConstants.SwerveModule.FRONT_RIGHT.canCoderId(), mFrontRight);

    mBackLeft =
        build(
            DriveConstants.SwerveModule.BACK_LEFT.canCoderId(),
            DriveConstants.SwerveModule.BACK_LEFT.encoderOffsetDegrees().getDegrees());
    mBackLeftObserver = new CanTsObserver(mBackLeft);
    cancoders.put(DriveConstants.SwerveModule.BACK_LEFT.canCoderId(), mBackLeft);

    mBackRight =
        build(
            DriveConstants.SwerveModule.BACK_RIGHT.canCoderId(),
            DriveConstants.SwerveModule.BACK_RIGHT.encoderOffsetDegrees().getDegrees());
    mBackRightObserver = new CanTsObserver(mBackRight);
    cancoders.put(DriveConstants.SwerveModule.BACK_RIGHT.canCoderId(), mBackRight);
  }

  public boolean allHaveBeenInitialized() {
    return mFrontLeftObserver.hasUpdate()
        && mFrontRightObserver.hasUpdate()
        && mBackLeftObserver.hasUpdate()
        && mBackRightObserver.hasUpdate();
  }

  public CANCoder getFrontLeft() {
    return mFrontLeft;
  }

  public CANCoder getFrontRight() {
    return mFrontRight;
  }

  public CANCoder getBackLeft() {
    return mBackLeft;
  }

  public CANCoder getBackRight() {
    return mBackRight;
  }

  public CANCoder get(int canId) {
    return cancoders.get(canId);
  }
}
