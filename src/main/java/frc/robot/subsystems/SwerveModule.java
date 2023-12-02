package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStickyFaults;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.constants.DriveConstants;
import frc.robot.util.CTREUtil;
import frc.robot.util.RevUtil;

/** A SwerveModules consists of a drive motor and a steer motor */
public class SwerveModule {

  private String moduleName;
  private CANSparkMax driveMotor;
  private RelativeEncoder driveEncoder;
  private CANSparkMax pivotMotor;
  private RelativeEncoder pivotMotorRelativeEncoder;
  private SparkMaxPIDController pivotMotorController;
  private CANCoder pivotAbsoluteEncoder;
  private Rotation2d referenceAngle = new Rotation2d();
  private boolean parked = false;
  private Rotation2d parkedDegrees;

  private static final double POSITION_CONVERSION_FACTOR =
      Math.PI * DriveConstants.WHEEL_DIAMETER_METERS * DriveConstants.DRIVE_GEAR_REDUCTION;

  private DoubleLogEntry driveMotorCurrent;
  private DoubleLogEntry driveMotorPosition;
  private DoubleLogEntry driveMotorVelocity;
  private DoubleLogEntry driveMotorAppliedOutput;
  private DoubleLogEntry driveMotorTemperature;

  private DoubleLogEntry pivotMotorCurrent;
  private DoubleLogEntry pivotMotorPosition;
  private DoubleLogEntry pivotMotorVelocity;
  private DoubleLogEntry pivotMotorAppliedOutput;
  private DoubleLogEntry pivotMotorTemperature;

  private DoubleLogEntry canCoderPosition;

  public Rotation2d getParkedDegrees() {
    return parkedDegrees;
  }

  public SwerveModule(
      DriveConstants.SwerveModule swerveModule, ShuffleboardContainer shuffleboard) {

    moduleName = swerveModule.moduleName();
    parkedDegrees = swerveModule.parkedDegrees();

    setupDriveMotor(swerveModule);
    setupCANCoder(swerveModule);
    setupPivotMotor(swerveModule);

    shuffleboard
        .addNumber(moduleName + " Cur degrees", () -> getSteerAngle().getDegrees())
        .withSize(2, 1);
    shuffleboard
        .addNumber(moduleName + " Target degrees", () -> getReferenceAngle().getDegrees())
        .withSize(2, 5);
    shuffleboard
        .addNumber(moduleName + " AbsEnc degrees", () -> getAbsoluteAngle().getDegrees())
        .withSize(2, 1);
    shuffleboard
        .addNumber(moduleName + " Ref Angle", () -> getReferenceAngle().getDegrees())
        .withSize(2, 1);

    shuffleboard.addNumber(moduleName + " Current Velocity", this::getDriveVelocity).withSize(2, 1);
    shuffleboard.addNumber(moduleName + " output", this::getDriveOutput).withSize(2, 1);
    shuffleboard.addNumber(moduleName + " Pos ", this::getDistance);

    DataLog log = DataLogManager.getLog();

    driveMotorCurrent = new DoubleLogEntry(log, "/drive/" + moduleName + "/drive_motor/current");
    driveMotorPosition = new DoubleLogEntry(log, "/drive/" + moduleName + "/drive_motor/current");
    driveMotorVelocity = new DoubleLogEntry(log, "/drive/" + moduleName + "/drive_motor/current");
    driveMotorAppliedOutput =
        new DoubleLogEntry(log, "/drive/" + moduleName + "/drive_motor/current");
    driveMotorTemperature =
        new DoubleLogEntry(log, "/drive/" + moduleName + "/drive_motor/current");

    pivotMotorCurrent = new DoubleLogEntry(log, "/drive/" + moduleName + "/pivot_motor/current");
    pivotMotorPosition = new DoubleLogEntry(log, "/drive/" + moduleName + "/pivot_motor/current");
    pivotMotorVelocity = new DoubleLogEntry(log, "/drive/" + moduleName + "/pivot_motor/current");
    pivotMotorAppliedOutput =
        new DoubleLogEntry(log, "/drive/" + moduleName + "/pivot_motor/current");
    pivotMotorTemperature =
        new DoubleLogEntry(log, "/drive/" + moduleName + "/pivot_motor/current");

    canCoderPosition = new DoubleLogEntry(log, "/drive/" + moduleName + "/pivot_motor/cancoder");
  }

  public void updateDataLogs() {
    driveMotorCurrent.append(driveMotor.getOutputCurrent());
    driveMotorPosition.append(driveEncoder.getPosition());
    driveMotorVelocity.append(driveEncoder.getVelocity());
    driveMotorAppliedOutput.append(driveMotor.getAppliedOutput());
    driveMotorTemperature.append(driveMotor.getMotorTemperature());

    pivotMotorCurrent.append(pivotMotor.getOutputCurrent());
    pivotMotorPosition.append(pivotMotorRelativeEncoder.getPosition());
    pivotMotorVelocity.append(pivotMotorRelativeEncoder.getVelocity());
    pivotMotorAppliedOutput.append(pivotMotor.getAppliedOutput());
    pivotMotorTemperature.append(pivotMotor.getMotorTemperature());

    canCoderPosition.append(getAbsoluteAngle().getDegrees());
  }

  /**
   * Setup a drive motor
   *
   * @param id The id of the drive motor
   */
  private void setupDriveMotor(DriveConstants.SwerveModule swerveModule) {
    driveMotor =
        new CANSparkMax(swerveModule.driveMotorPort(), CANSparkMaxLowLevel.MotorType.kBrushless);
    // driveMotor.restoreFactoryDefaults();
    driveMotor.setInverted(swerveModule.driveMotorInverted());

    // Setup voltage compensation
    RevUtil.checkRevError(
        driveMotor.enableVoltageCompensation(DriveConstants.NOMINAL_VOLTAGE),
        "Failed to enable voltage compensation");
    RevUtil.checkRevError(
        driveMotor.setSmartCurrentLimit(DriveConstants.DRIVE_CURRENT_LIMIT),
        "Failed to set current limit for NEO");

    RevUtil.setPeriodicFramePeriodHigh(driveMotor, moduleName + " Drive Motor");

    // Set neutral mode to brake
    driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Setup encoder
    driveEncoder = driveMotor.getEncoder();

    driveEncoder.setPosition(0.0);
    // RevUtil.checkRevError(
    //    driveEncoder.setPositionConversionFactor(0.0), "drive encoder set position conversion");

    // RevUtil.checkRevError(
    //    driveEncoder.setVelocityConversionFactor(0.0), "Drive encoder
    // setVelocityConversionFactor");
    // driveEncoder.setVelocityConversionFactor(POSITION_CONVERSION_FACTOR / 60.0);
    driveMotor.burnFlash(); // might not work, needs a delay after setting values
  }

  /**
   * Setup a pivot motor
   *
   * @param id The id of the steer motor
   */
  public void setupPivotMotor(DriveConstants.SwerveModule swerveModule) {
    double pidProportional = .9;
    double pidIntegral = 0.0;
    double pidDerivative = 0.0; // 0.1;

    pivotMotor =
        new CANSparkMax(swerveModule.pivotMotorPort(), CANSparkMaxLowLevel.MotorType.kBrushless);
    pivotMotor.setInverted(swerveModule.pivotMotorInverted());

    RevUtil.setPeriodicFramePeriodLow(pivotMotor, moduleName + " Drive Motor");

    RevUtil.checkRevError(
        pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake), "Failed to set NEO idle mode");
    RevUtil.checkRevError(
        pivotMotor.enableVoltageCompensation(DriveConstants.NOMINAL_VOLTAGE),
        "Failed to enable voltage compensation");
    RevUtil.checkRevError(
        pivotMotor.setSmartCurrentLimit(DriveConstants.STEER_CURRENT_LIMIT),
        "Failed to set NEO current limits");

    pivotMotorRelativeEncoder = pivotMotor.getEncoder();

    RevUtil.checkRevError(
        pivotMotorRelativeEncoder.setPositionConversionFactor(
            2.0 * Math.PI * DriveConstants.STEER_DRIVE_REDUCTION),
        "Failed to set NEO encoder position conversion factor");
    RevUtil.checkRevError(
        pivotMotorRelativeEncoder.setVelocityConversionFactor(
            2.0 * Math.PI * DriveConstants.STEER_DRIVE_REDUCTION / 60.0),
        "Failed to set NEO encoder velocity conversion factor");

    RevUtil.checkRevError(
        pivotMotorRelativeEncoder.setPosition(getAbsoluteAngle().getRadians()),
        "Failed to set NEO encoder position");

    pivotMotorController = pivotMotor.getPIDController();

    RevUtil.checkRevError(
        pivotMotorController.setP(pidProportional), "Failed to set NEO PID proportional constant");
    RevUtil.checkRevError(
        pivotMotorController.setI(pidIntegral), "Failed to set NEO PID integral constant");
    RevUtil.checkRevError(
        pivotMotorController.setD(pidDerivative), "Failed to set NEO PID derivative constant");
    RevUtil.checkRevError(
        pivotMotorController.setFF(0.0), "Failed to set NEO PID derivative constant");
    RevUtil.checkRevError(
        pivotMotorController.setPositionPIDWrappingEnabled(true), "Failed to set NEO PID wrapping");
    RevUtil.checkRevError(
        pivotMotorController.setPositionPIDWrappingMinInput(0.0), "Failed to set NEO PID wrapping");
    RevUtil.checkRevError(
        pivotMotorController.setPositionPIDWrappingMaxInput(2.0 * Math.PI),
        "Failed to set NEO PID wrapping");
    RevUtil.checkRevError(
        pivotMotorController.setFeedbackDevice(pivotMotorRelativeEncoder),
        "Failed to set NEO PID feedback device");

    RevUtil.checkRevError(pivotMotor.burnFlash(), "Failed to burn flash on pivot motor");
  }

  /**
   * Setup a CAN coder
   *
   * @param id The id of the CAN coder
   * @param offset the offset of the magnet
   */
  public void setupCANCoder(DriveConstants.SwerveModule swerveModule) {
    // CANCoderConfiguration config = new CANCoderConfiguration();
    // config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    // config.magnetOffsetDegrees = swerveModule.encoderOffsetDegrees().getDegrees();
    // config.sensorDirection = false;

    // pivotAbsoluteEncoder = new CANCoder(swerveModule.canCoderId());
    // CTREUtil.checkCtreError(
    //    pivotAbsoluteEncoder.configAllSettings(config, 250), "Failed to configure CANCoder");
    // CTREUtil.setPeriodicFrameRate(pivotAbsoluteEncoder);

    pivotAbsoluteEncoder = Cancoders.getInstance().get(swerveModule.canCoderId());
  }

  /**
   * A Rotation2d representation of the angle of the steer absolute encoder
   *
   * @return The angle
   */
  public Rotation2d getAbsoluteAngle() {
    double angle =
        MathUtil.inputModulus(
            Math.toRadians(pivotAbsoluteEncoder.getAbsolutePosition()), 0.0, 2.0 * Math.PI);

    CTREUtil.checkCtreError(pivotAbsoluteEncoder.getLastError(), "Last CANCoder Error");

    CANCoderStickyFaults faults = new CANCoderStickyFaults();
    CTREUtil.checkCtreError(
        pivotAbsoluteEncoder.getStickyFaults(faults), " Error getting sticky faults");

    if (faults.hasAnyFault()) {
      DriverStation.reportError("CANCoder fault: " + faults.toString(), false);
      pivotAbsoluteEncoder.clearStickyFaults();
    }

    return Rotation2d.fromRadians(angle);
  }

  /**
   * Returns a Rotation2d representation of the reference angle
   *
   * @return The angle
   */
  public Rotation2d getReferenceAngle() {
    return referenceAngle;
  }

  /**
   * Sets the reference angle of the steer motor controller
   *
   * @param referenceAngle The reference angle
   */
  public void setReferenceAngle(Rotation2d referenceAngle) {
    pivotMotorController.setReference(
        MathUtil.inputModulus(referenceAngle.getRadians(), 0, 2.0 * Math.PI),
        CANSparkMax.ControlType.kPosition);

    this.referenceAngle = referenceAngle;
  }

  /**
   * Returns the current steer angle
   *
   * @return Rotation2d of the angle
   */
  public Rotation2d getSteerAngle() {
    double motorAngleRadians =
        MathUtil.inputModulus(pivotMotorRelativeEncoder.getPosition(), 0.0, 2.0 * Math.PI);

    return Rotation2d.fromRadians(motorAngleRadians);
  }

  /**
   * Returns the current drive veloticty
   *
   * @return The veloticty
   */
  public double getDriveVelocity() {
    return driveEncoder.getVelocity() * POSITION_CONVERSION_FACTOR / 60;
  }

  public double getDriveOutput() {
    return driveMotor.getAppliedOutput();
  }

  /**
   * Returns a Rotation2d representation of the position of the drive encoder
   *
   * @return The position
   */
  public double getDistance() {
    return driveEncoder.getPosition() * POSITION_CONVERSION_FACTOR;
  }

  /**
   * Returns the position of the swerve module
   *
   * @return The position
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDistance(), getSteerAngle());
  }

  /**
   * Returns the position of the swerve module
   *
   * @return The position
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getSteerAngle());
  }

  /**
   * Returns the position of the steer motor's relative encoder
   *
   * @return The position
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(pivotMotorRelativeEncoder.getPosition());
  }

  public void setParkMode(boolean mode) {
    parked = mode;
  }

  /**
   * Sets the steer angle in radians
   *
   * @param state The state of the swerve module
   */
  public void set(SwerveModuleState state) {
    if (parked) {
      return;
    }

    state = SwerveModuleState.optimize(state, getSteerAngle());

    driveMotor.setVoltage(
        state.speedMetersPerSecond
            / DriveConstants.MAX_SPEED_METERS_PER_SECOND
            * DriveConstants.MAX_VOLTAGE);
    setReferenceAngle(state.angle);
  }
}
