package frc.robot.subsystems;

import static frc.constants.ManipulatorConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.CubeHuntCommand;
import frc.robot.util.RevUtil;
import java.util.HashMap;

public class ManipulatorSubystem extends SubsystemBase {
  private final DoubleSolenoid manipulatorSolenoid;

  private final CANSparkMax elevatorMotor =
      new CANSparkMax(ELEVATOR_MOTOR_PORT, MotorType.kBrushless);
  private final CANSparkMax elevatorMotor_follower =
      new CANSparkMax(ELEVATOR_MOTOR_FOLLOWER_PORT, MotorType.kBrushless);
  private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  private final DigitalInput elevatorLimitSwitch = new DigitalInput(ELEVATOR_LIMIT_SWITCH_PORT);

  private final CANSparkMax drawerMotor = new CANSparkMax(DRAWER_MOTOR_PORT, MotorType.kBrushless);
  private final RelativeEncoder drawerEncoder = drawerMotor.getEncoder();
  private final DigitalInput drawerLimitSwitch = new DigitalInput(DRAWER_LIMIT_SWITCH_PORT);

  private final CANSparkMax wristMotor = new CANSparkMax(WRIST_MOTOR_PORT, MotorType.kBrushless);
  private final SparkMaxAbsoluteEncoder wristEncoder =
      wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private final CANSparkMax rollerMotor = new CANSparkMax(ROLLER_MOTOR_PORT, MotorType.kBrushless);
  private final AnalogInput rollerSensor = new AnalogInput(ROLLER_SENSOR_ANALOG_PORT);

  private SparkMaxPIDController elevatorPIDController;
  private SparkMaxPIDController drawerPIDController;
  private SparkMaxPIDController wristPIDController;

  private double elevatorSetPoint;
  private double drawerSetPoint;
  private double wristSetPoint;

  private double elevatorSpeed = 0.0;
  private double drawerSpeed = 0.0;

  DoubleLogEntry elevatorPositionLog;
  DoubleLogEntry drawerPositionLog;
  DoubleLogEntry wristPositionLog;
  BooleanLogEntry clawPositionLog;

  public ManipulatorSubystem(PneumaticHub pneumaticHub) {
    manipulatorSolenoid = pneumaticHub.makeDoubleSolenoid(0, 1);

    DataLog log = DataLogManager.getLog();
    elevatorPositionLog = new DoubleLogEntry(log, "/manipulator/elevator/position");
    drawerPositionLog = new DoubleLogEntry(log, "/manipulator/drawer/position");
    wristPositionLog = new DoubleLogEntry(log, "/manipulator/wrist/position");
    clawPositionLog = new BooleanLogEntry(log, "/manipulator/claw/position");

    setupManipulatorMotor(elevatorMotor, "Elevator");
    elevatorMotor_follower.follow(elevatorMotor, true);
    elevatorMotor.setInverted(ELEVATOR_MOTOR_REVERSED);

    setupRelativeEncoder(elevatorEncoder, "Elevator");

    elevatorPIDController = elevatorMotor.getPIDController();
    setElevatorPIDCoefficients();

    RevUtil.checkRevError(
        elevatorPIDController.setFeedbackDevice(elevatorEncoder),
        "Elevator Failed to set NEO PID feedback device");

    setupManipulatorMotor(drawerMotor, "Drawer");
    drawerMotor.setInverted(DRAWER_MOTOR_REVERSED);

    setupRelativeEncoder(drawerEncoder, "Drawer");

    drawerPIDController = drawerMotor.getPIDController();
    drawerPIDController.setP(DRAWER_PID_P);
    drawerPIDController.setI(DRAWER_PID_I);
    drawerPIDController.setD(DRAWER_PID_D);
    drawerPIDController.setFF(DRAWER_PID_FF);
    drawerPIDController.setIZone(DRAWER_PID_I_ZONE);
    RevUtil.checkRevError(
        drawerPIDController.setFeedbackDevice(drawerEncoder),
        "Drawer Failed to set NEO PID feedback device");

    setupManipulatorMotor(wristMotor, "Wrist");
    RevUtil.checkRevError(
        wristMotor.setIdleMode(IdleMode.kBrake), "[Wrist] Error setting brake mode.");
    wristMotor.setInverted(WRIST_MOTOR_REVERSED);
    setupAbsoluteEncoder(wristEncoder, "Wrist");

    wristPIDController = wristMotor.getPIDController();
    wristPIDController.setP(WRIST_PID_P);
    wristPIDController.setI(WRIST_PID_I);
    wristPIDController.setD(WRIST_PID_D);
    wristPIDController.setFF(WRIST_PID_FF);
    wristPIDController.setIZone(WRIST_PID_I_ZONE);
    RevUtil.checkRevError(
        wristPIDController.setFeedbackDevice(wristEncoder),
        "Wrist Failed to set NEO PID feedback device");

    setupManipulatorMotor(rollerMotor, "Roller");

    setupShuffleboardTab();
  }

  private void setElevatorPIDCoefficients() {
    RevUtil.checkRevError(elevatorPIDController.setP(ELEVATOR_PID_UP_P, 0), "Error setting P");
    RevUtil.checkRevError(elevatorPIDController.setI(ELEVATOR_PID_UP_I, 0), "Error setting I");
    RevUtil.checkRevError(elevatorPIDController.setD(ELEVATOR_PID_UP_D, 0), "Error setting D up");
    RevUtil.checkRevError(
        elevatorPIDController.setIZone(ELEVATOR_PID_UP_I_ZONE, 0), "Error setting izone");

    RevUtil.checkRevError(elevatorPIDController.setP(ELEVATOR_PID_DOWN_P, 1), "Error setting P");
    RevUtil.checkRevError(elevatorPIDController.setI(ELEVATOR_PID_DOWN_I, 1), "Error setting I");
    RevUtil.checkRevError(
        elevatorPIDController.setD(ELEVATOR_PID_DOWN_D, 1), "Error setting D down");
    RevUtil.checkRevError(
        elevatorPIDController.setIZone(ELEVATOR_PID_DOWN_I_ZONE, 1), "error setting izone");
  }

  private void setupShuffleboardTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Manipulator");
    tab.addNumber("Elevator Amps", elevatorMotor::getOutputCurrent).withPosition(0, 0);
    tab.addNumber("Elevator Pos", this::getElevatorPosition).withPosition(1, 0);
    tab.addNumber("Elevator Set", this::getElevatorSetPoint).withPosition(2, 0);
    tab.addNumber("Elevator Speed", this::getElevatorSpeed).withPosition(3, 0);
    tab.addNumber("Elevator Output", elevatorMotor::getAppliedOutput).withPosition(4, 0);
    tab.addNumber("Elevator Temp", elevatorMotor::getMotorTemperature).withPosition(5, 0);

    tab.addNumber("Drawer Amps", drawerMotor::getOutputCurrent).withPosition(0, 1);
    tab.addNumber("Drawer Pos", this::getDrawerPosition).withPosition(1, 1);
    tab.addNumber("Drawer Set", this::getDrawerSetPoint).withPosition(2, 1);
    tab.addNumber("Drawer Speed", this::getDrawerSpeed).withPosition(3, 1);
    tab.addNumber("Drawer Output", drawerMotor::getAppliedOutput).withPosition(4, 1);
    tab.addNumber("Drawer Temp", drawerMotor::getMotorTemperature).withPosition(5, 1);

    tab.addNumber("Wrist Amps", wristMotor::getOutputCurrent).withPosition(0, 2);
    tab.addNumber("Wrist Pos", this::getWristPosition).withPosition(1, 2);
    tab.addNumber("Wrist Set", this::getWristSetPoint).withPosition(2, 2);
    tab.addNumber("Wrist Output", wristMotor::getAppliedOutput).withPosition(3, 2);
    tab.addNumber("Wrist Temp", wristMotor::getMotorTemperature).withPosition(4, 2);

    tab.addBoolean("Wrist Safe", this::isWristInSafeZone).withPosition(5, 2);
    tab.addBoolean("Elevator Home", this::isElevatorLimitSwitchActive).withPosition(6, 2);
    tab.addBoolean("Drawer Home", this::isDrawerLimitSwitchActive).withPosition(7, 2);
    tab.add("Calib Drawer", getDrawerCalibrateCommand()).withPosition(8, 2);

    tab.addNumber("Roller Sensor", this::getRollerSensor).withPosition(7, 1);
    tab.addBoolean("Solenoid State", this::getClawOpenStatus).withPosition(8, 1);

    tab.addNumber("Roller Amps", rollerMotor::getOutputCurrent).withPosition(0, 3);
    tab.addNumber("Roller Output", rollerMotor::getAppliedOutput).withPosition(3, 3);
    tab.addNumber("Roller Temp", rollerMotor::getMotorTemperature).withPosition(4, 3);
  }

  public boolean getClawOpenStatus() {
    return manipulatorSolenoid.get() == DoubleSolenoid.Value.kReverse;
  }

  @Override
  public void periodic() {
    updateDataLogs();
  }

  private void updateDataLogs() {
    elevatorPositionLog.append(getElevatorPosition());
    drawerPositionLog.append(getDrawerPosition());
    wristPositionLog.append(getWristPosition());
    clawPositionLog.append(getClawOpenStatus());
  }

  public HashMap<String, Command> getEventMap() {

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("lowerWrist", getFloorPickupPositionCommand());
    eventMap.put(
        "startRollers",
        new InstantCommand(
            () -> {
              release();
              rollerGrab();
            }));

    eventMap.put(
        "lowerAndStartRollers",
        new InstantCommand(
                () -> {
                  release();
                  rollerGrab();
                })
            .andThen(getFloorPickupPositionCommand()));

    return eventMap;
  }

  public double getRollerSensor() {
    return rollerSensor.getVoltage();
  }

  private void setupManipulatorMotor(CANSparkMax motor, String description) {

    RevUtil.checkRevError(
        motor.enableVoltageCompensation(NOMINAL_VOLTAGE),
        "[" + description + "] Failed to enable voltage compensation");
    RevUtil.checkRevError(
        motor.setSmartCurrentLimit(CURRENT_LIMIT),
        "[" + description + "] Failed to set NEO current limits");

    RevUtil.checkRevError(
        motor.setClosedLoopRampRate(0.1),
        "[" + description + "] Error setting closed loop ramp rate.");
    RevUtil.checkRevError(
        motor.setIdleMode(IdleMode.kBrake), "[" + description + "] Error setting brake mode.");
    RevUtil.setPeriodicFramePeriodLow(motor, description);
  }

  private void setupRelativeEncoder(RelativeEncoder encoder, String description) {
    RevUtil.checkRevError(
        encoder.setPositionConversionFactor(1),
        "[" + description + "] Failed to set NEO encoder position conversion factor");
    RevUtil.checkRevError(
        encoder.setVelocityConversionFactor(1),
        "[" + description + "] Failed to set NEO encoder velocity conversion factor");

    RevUtil.checkRevError(
        encoder.setPosition(1.0), "[" + description + "] Failed to set NEO encoder position");
  }

  private void setupAbsoluteEncoder(SparkMaxAbsoluteEncoder encoder, String description) {
    RevUtil.checkRevError(
        encoder.setPositionConversionFactor(1),
        "[" + description + "] Failed to set NEO encoder position conversion factor");
    RevUtil.checkRevError(
        encoder.setVelocityConversionFactor(1),
        "[" + description + "] Failed to set NEO encoder velocity conversion factor");
  }

  /////////////////////////////////////////////////////////////////////////
  // teleop commands
  /////////////////////////////////////////////////////////////////////////

  public Command lowerPositionCommand() {
    return new SequentialCommandGroup(
        setWristCommand(WRIST_SAFE_MIN + .05),
        new WaitUntilCommand(() -> isWristInSafeZone()),
        setElevatorCommand(ELEVATOR_HOME),
        setDrawerCommand(DRAWER_MIN),
        setWristCommand(WRIST_STRAIGHT));
  }

  public Command middleCubePositionCommand() {
    return new SequentialCommandGroup(
        new ConditionalCommand(
            new InstantCommand(), setWristCommand(WRIST_SAFE_MIN + .05), () -> isWristInSafeZone()),
        new WaitUntilCommand(() -> isWristInSafeZone()),
        setDrawerCommand(DRAWER_MIN),
        setElevatorCommand(ELEVATOR_MID),
        new WaitUntilCommand(() -> isElevatorAtSetPoint()),
        setWristCommand(WRIST_LOWER), // WRIST_FOR_MID_POS),
        new WaitUntilCommand(() -> isWristAtSetPoint()));
  }

  public Command middleConePositionCommand() {
    return new SequentialCommandGroup(
        new ConditionalCommand(
            new InstantCommand(), setWristCommand(WRIST_SAFE_MIN + .05), () -> isWristInSafeZone()),
        new WaitUntilCommand(() -> isWristInSafeZone()),
        setDrawerCommand(CONE_DRAWER_MID),
        setElevatorCommand(ELEVATOR_MID_CONE),
        new WaitUntilCommand(() -> isElevatorAtSetPoint()),
        setWristCommand(WRIST_FOR_MID_POS),
        new WaitUntilCommand(() -> isWristAtSetPoint()));
  }

  public Command getMidElevatorCommand() {
    return new SequentialCommandGroup(
        new ConditionalCommand(
            new InstantCommand(), setWristCommand(WRIST_SAFE_MIN + .05), () -> isWristInSafeZone()),
        new WaitUntilCommand(() -> isWristInSafeZone()),
        setElevatorCommand(ELEVATOR_SHELF));
  }

  public Command upperCubePositionCommand() {
    return new SequentialCommandGroup(
        new ConditionalCommand(
            new InstantCommand(), setWristCommand(WRIST_SAFE_MIN + .05), () -> isWristInSafeZone()),
        new WaitUntilCommand(() -> isWristInSafeZone()),
        setDrawerCommand(DRAWER_HIGH_CUBE),
        setElevatorCommand(ELEVATOR_MAX),
        new WaitUntilCommand(() -> isElevatorAtSetPoint()),
        setWristCommand(WRIST_STRAIGHT),
        new WaitUntilCommand(() -> isWristAtSetPoint()));
  }

  public Command upperConePositionCommand() {
    return new SequentialCommandGroup(
        new ConditionalCommand(
            new InstantCommand(), setWristCommand(WRIST_SAFE_MIN + .05), () -> isWristInSafeZone()),
        new WaitUntilCommand(() -> isWristInSafeZone()),
        setDrawerCommand(DRAWER_HIGH_CUBE),
        setElevatorCommand(ELEVATOR_MAX),
        new WaitUntilCommand(() -> isElevatorAtSetPoint()),
        setWristCommand(WRIST_TOP_SCORE_POSITION),
        new WaitUntilCommand(() -> isWristAtSetPoint()));
  }

  public Command getShelfPickupCommand() {
    return new SequentialCommandGroup(
        new ConditionalCommand(
            new InstantCommand(), setWristCommand(WRIST_SAFE_MIN + .05), () -> isWristInSafeZone()),
        new WaitUntilCommand(() -> isWristInSafeZone()),
        setDrawerCommand(DRAWER_SHELF),
        setElevatorCommand(ELEVATOR_SHELF),
        new WaitUntilCommand(() -> isElevatorAtSetPoint()),
        setWristCommand(WRIST_SHELF));
  }

  /**
   * Command will position for the middle, then release, then home
   *
   * @return
   */
  public Command getPositionAndScoreMiddleCommand() {
    SequentialCommandGroup command =
        new SequentialCommandGroup(
            new InstantCommand(this::grab),
            middleConePositionCommand(),
            new WaitCommand(0.25),
            new InstantCommand(this::release),
            getFastHomeWristUpCommand());
    command.addRequirements(this);
    return command;
  }

  public Command getPositionAndScoreHighCommand() {
    SequentialCommandGroup command =
        new SequentialCommandGroup(
            new InstantCommand(this::grab),
            upperConePositionCommand(),
            new WaitCommand(0.25),
            new InstantCommand(() -> rollerDropVerySlow()),
            new WaitCommand(2.5),
            new InstantCommand(() -> rollerStop()),
            new InstantCommand(this::release),
            getFastHomeWristUpCommand());
    command.addRequirements(this);
    return command;
  }

  // position for floor pickup
  public Command getFloorPickupPositionCommand() {
    return new SequentialCommandGroup(
        new ConditionalCommand(
            new InstantCommand(), setWristCommand(WRIST_SAFE_MIN + .05), () -> isWristInSafeZone()),
        new WaitUntilCommand(() -> isWristInSafeZone()),
        setElevatorCommand(ELEVATOR_HOME),
        setDrawerCommand(DRAWER_MIN),
        setWristCommand(WRIST_LOWER));
  }

  public Command getFloorPickupCubeCommand() {
    return new SequentialCommandGroup(
        // drive to position
        new InstantCommand(() -> rollerGrab()),
        new WaitCommand(0.5), // inspect sensor to see if we have the game piece?
        new InstantCommand(() -> grab()),
        new InstantCommand(() -> rollerStop()));
  }

  public Command getShootCubeCommand() {
    SequentialCommandGroup command =
        new SequentialCommandGroup(
            new InstantCommand(() -> grab()),
            new WaitCommand(.2),
            new InstantCommand(() -> rollerDrop()),
            new WaitCommand(.2),
            new InstantCommand(() -> rollerStop()));
    command.addRequirements(this);
    return command;
  }

  public Command getShootMidCubeCommand() {
    SequentialCommandGroup command =
        new SequentialCommandGroup(
            new InstantCommand(() -> grab()),
            new WaitCommand(.2),
            new InstantCommand(() -> rollerDropSlower()),
            new WaitCommand(.1),
            new InstantCommand(() -> rollerStop()));
    command.addRequirements(this);
    return command;
  }

  public Command getHomeCommand() {
    return new SequentialCommandGroup(
        // new ConditionalCommand(
        //     new InstantCommand(), setWristCommand(WRIST_SAFE_MIN + .05), () ->
        // isWristInSafeZone()),
        setWristCommand(WRIST_SAFE_MIN + .05),
        new WaitUntilCommand(() -> isWristInSafeZone()),
        setElevatorCommand(ELEVATOR_HOME),
        setDrawerCommand(DRAWER_HOME),
        new WaitUntilCommand(() -> isDrawerAtSetPoint()),
        new WaitUntilCommand(() -> isElevatorInSafeZone()),
        getDrawerCalibrateCommand(),
        setWristCommand(WRIST_MIN));
  }

  public Command getFastHomeWristUpCommand() {
    return new SequentialCommandGroup(
        setWristCommand(WRIST_SAFE_MIN + .05),
        new WaitUntilCommand(() -> isWristInSafeZone()),
        setElevatorCommand(ELEVATOR_HOME),
        setDrawerCommand(DRAWER_HOME),
        // new WaitUntilCommand(() -> isDrawerAtSetPoint()),
        new WaitUntilCommand(() -> isElevatorInSafeZone()));
  }

  public Command getWristUpCommand() {
    return setWristCommand(WRIST_SAFE_MIN + .05);
  }

  public Command getWristMinCommand() {
    return setWristCommand(WRIST_MIN);
  }

  public Command getWristFloorCommand() {
    return setWristCommand(WRIST_LOWER);
  }

  public Command getDrawerMaxCommand() {
    return setDrawerCommand(DRAWER_SHELF);
  }

  public void grab() {
    manipulatorSolenoid.set(Value.kForward);
  }

  public void release() {
    manipulatorSolenoid.set(Value.kReverse);
  }

  public void rollerGrab() {
    DataLogManager.log("running roller grab " + ROLLER_GRAB_SPEED);
    rollerMotor.setVoltage(ROLLER_GRAB_SPEED * NOMINAL_VOLTAGE);
    DataLogManager.log("running roller grab " + ROLLER_GRAB_SPEED);
  }

  public void rollerDrop() {
    rollerMotor.setVoltage(ROLLER_DROP_SPEED * NOMINAL_VOLTAGE);
  }

  public void rollerDropSlower() {
    rollerMotor.setVoltage(-.8 * NOMINAL_VOLTAGE);
  }

  public void rollerDropVerySlow() {
    rollerMotor.setVoltage(-.1 * NOMINAL_VOLTAGE);
  }

  public void rollerStop() {
    rollerMotor.stopMotor();
  }

  public Command getIntakeObjectCommand() {
    SequentialCommandGroup command =
        new SequentialCommandGroup(
            new InstantCommand(this::rollerGrab),
            new WaitCommand(0.5),
            new InstantCommand(this::rollerStop));
    command.addRequirements(this);

    return command;
  }

  public Command getIntakeObjectCommand(DriveSubsystem driveSubsystem) {
    SequentialCommandGroup command =
        new SequentialCommandGroup(
            new InstantCommand(this::rollerGrab),
            new InstantCommand(this::release),
            new CubeHuntCommand(driveSubsystem, this::getRollerSensor),
            new InstantCommand(this::rollerStop));

    command.addRequirements(this);

    return command;
  }

  public boolean isRollerSensorActive() {
    return rollerSensor.getAverageValue() > 4;
  }

  public double getElevatorPosition() {
    return elevatorEncoder.getPosition();
  }

  public double getMaxElevatorPosition() {
    return ELEVATOR_MAX;
  }

  private double getDrawerPosition() {
    return drawerEncoder.getPosition();
  }

  private double getWristPosition() {
    double position = wristEncoder.getPosition();
    // if (position > WRIST_MAX + 0.1) {
    //   position = WRIST_MIN;
    // }
    return position;
  }

  public void setElevatorReference(double position) {
    System.out.println("elevator reference:" + position);
    elevatorSetPoint = MathUtil.clamp(position, ELEVATOR_MIN, ELEVATOR_MAX);
    System.out.println("elevator setPoint:" + elevatorSetPoint);
    int pidSlot = 0;
    if (elevatorSetPoint == ELEVATOR_HOME) {
      pidSlot = 1;
    }
    elevatorPIDController.setReference(
        elevatorSetPoint, CANSparkMax.ControlType.kPosition, pidSlot);
  }

  public Command setElevatorCommand(double position) {
    return new InstantCommand(() -> setElevatorReference(position));
  }

  public double getElevatorSetPoint() {
    return elevatorSetPoint;
  }

  public void homeElevator() {}

  public void setDrawerReference(double position) {
    drawerSetPoint = MathUtil.clamp(position, DRAWER_MIN, DRAWER_MAX);
    drawerPIDController.setReference(drawerSetPoint, CANSparkMax.ControlType.kPosition);
  }

  public Command setDrawerCommand(double setPoint) {
    return new InstantCommand(() -> setDrawerReference(setPoint));
  }

  public double getDrawerSetPoint() {
    return drawerSetPoint;
  }

  public void setWristReference(double position) {
    wristSetPoint = MathUtil.clamp(position, WRIST_MIN, WRIST_MAX);
    /*System.out.println(
    "setWristReference  pos:["
        + position
        + "]  setPoint:["
        + wristSetPoint
        + "] currentPosition:["
        + getWristPosition()
        + "]")*/
    wristPIDController.setReference(wristSetPoint, CANSparkMax.ControlType.kPosition);
  }

  public Command setWristCommand(double position) {
    return new InstantCommand(() -> setWristReference(position));
  }

  public double getWristSetPoint() {
    return wristSetPoint;
  }

  public double getElevatorSpeed() {
    return elevatorSpeed;
  }

  public void runElevator(double speed) {
    elevatorSpeed = speed;
    if ((getElevatorPosition() < ELEVATOR_MAX && speed > 0.0)
        || (getElevatorPosition() > ELEVATOR_MIN && speed < 0.0)) {
      elevatorMotor.setVoltage(speed * NOMINAL_VOLTAGE);
    } else {
      elevatorMotor.stopMotor();
    }
  }

  public void raiseElevator() {
    if (getElevatorPosition() <= ELEVATOR_MAX) {
      elevatorMotor.setVoltage(RAISE_ELEVATOR_SPEED * 12.0);
    } else {
      elevatorMotor.setVoltage(0);
    }
  }

  public void lowerElevator() {
    if (getElevatorPosition() >= ELEVATOR_MIN) {
      elevatorMotor.setVoltage(LOWER_ELEVATOR_SPEED * 12.0);
    } else {
      stopElevator();
    }
  }

  public void stopElevator() {
    elevatorMotor.stopMotor();
  }

  public double getDrawerSpeed() {
    return drawerSpeed;
  }

  public void runDrawer(double speed) {
    drawerSpeed = speed;
    if ((getDrawerPosition() < DRAWER_MAX && speed > 0.0)
        || (getDrawerPosition() > DRAWER_MIN && speed < 0.0)) {
      drawerMotor.setVoltage(speed * NOMINAL_VOLTAGE);
    } else {
      stopDrawer();
    }
  }

  public void runDrawerUnsafe(double speed) {
    drawerSpeed = speed;
    drawerMotor.setVoltage(speed * NOMINAL_VOLTAGE);
  }

  public void extendDrawer() {
    if (getDrawerPosition() <= DRAWER_MAX) {

      drawerMotor.set(EXTEND_DRAWER_SPEED * NOMINAL_VOLTAGE);
    } else {
      stopDrawer();
    }
  }

  public void retractDrawer() {
    if (getDrawerPosition() >= DRAWER_MIN) {

      drawerMotor.set(RETRACT_DRAWER_SPEED * NOMINAL_VOLTAGE);
    } else {
      stopDrawer();
    }
  }

  public void stopDrawer() {
    drawerMotor.stopMotor();
  }

  public void extendDrawerToMax() {
    System.out.println("Extending drawer to " + DRAWER_MAX);
    drawerPIDController.setReference(DRAWER_MAX, CANSparkMax.ControlType.kPosition);
  }

  public void retractDrawerToMin() {
    System.out.println("Retracting drawer to " + DRAWER_MIN);

    drawerPIDController.setReference(DRAWER_MIN, CANSparkMax.ControlType.kPosition);
  }

  public void flickUpWrist() {
    if (getWristPosition() <= WRIST_MAX) {
      wristMotor.set(FLICK_WRIST_UP_SPEED * NOMINAL_VOLTAGE);
    } else {
      stopWrist();
    }
  }

  public void flickDownWrist() {
    if (getWristPosition() >= WRIST_MIN) {
      wristMotor.setVoltage(FLICK_WRIST_DOWN_SPEED * NOMINAL_VOLTAGE);
    } else {
      stopWrist();
    }
  }

  public void stopWrist() {
    wristMotor.stopMotor();
  }

  public boolean isWristInSafeZone() {
    return (getWristPosition() > WRIST_SAFE_MIN) && (getWristPosition() < WRIST_SAFE_MAX);
  }

  public boolean isWristAtSetPoint() {
    return Math.abs(getWristPosition() - wristSetPoint) < .05;
  }

  public boolean isElevatorInSafeZone() {
    return (getElevatorPosition() < ELEVATOR_SAFE_MAX);
  }

  public boolean isElevatorAtSetPoint() {
    return Math.abs(getElevatorPosition() - elevatorSetPoint) < 10.0;
  }

  public boolean isDrawerAtSetPoint() {
    return Math.abs(getDrawerPosition() - drawerSetPoint) < 6.0;
  }

  public boolean isElevatorAlmostUp() {
    return getElevatorPosition() > ELEVATOR_ALMOST_UP;
  }

  public void setElevatorHeightinches(double inches) {}

  public void runRoller(double speed) {
    rollerMotor.setVoltage(speed * NOMINAL_VOLTAGE);
  }

  public void runRollerDefault(double speedIn, double speedOut) {
    double factor = 1.0;
    if (speedIn > 0 && speedOut == 0) {
      rollerMotor.setVoltage(speedIn * factor * NOMINAL_VOLTAGE);
    } else if (speedIn == 0 && speedOut > 0) {
      if (getElevatorPosition() > 2) {
        factor = 0.1;
      }
      rollerMotor.setVoltage(-speedOut * factor * NOMINAL_VOLTAGE);
    } else {
      rollerMotor.setVoltage(0.0);
    }
  }

  public boolean isDrawerLimitSwitchActive() {
    return drawerLimitSwitch.get();
  }

  public boolean isElevatorLimitSwitchActive() {
    return elevatorLimitSwitch.get();
  }

  public void resetElevatorEncoder() {
    elevatorEncoder.setPosition(ELEVATOR_MIN);
  }

  public void resetDrawerEncoder() {
    drawerEncoder.setPosition(DRAWER_MIN);
  }

  public Command getElevatorCalibrateCommand() {
    Command command =
        new ConditionalCommand(
            new InstantCommand(),
            new SequentialCommandGroup(
                setWristCommand(WRIST_SAFE_MIN + .05),
                new WaitUntilCommand(this::isWristInSafeZone),
                new InstantCommand(() -> runElevator(-.1)),
                new WaitUntilCommand(this::isElevatorLimitSwitchActive),
                new InstantCommand(() -> runElevator(0.0)),
                new InstantCommand(this::resetElevatorEncoder),
                setElevatorCommand(ELEVATOR_MIN)),
            this::isElevatorLimitSwitchActive);
    return command;
  }

  public CommandBase getDrawerCalibrateCommand() {
    CommandBase command =
        new ConditionalCommand(
            new InstantCommand(),
            new SequentialCommandGroup(
                setWristCommand(WRIST_SAFE_MIN + .03),
                new WaitUntilCommand(this::isWristInSafeZone),
                new InstantCommand(() -> runDrawerUnsafe(-.1)),
                new WaitUntilCommand(this::isDrawerLimitSwitchActive),
                new InstantCommand(() -> runDrawerUnsafe(0.0)),
                new InstantCommand(this::resetDrawerEncoder),
                setDrawerCommand(DRAWER_MIN),
                setWristCommand(WRIST_MIN)),
            this::isDrawerLimitSwitchActive);
    return command;
  }

  public void adjustElevatorHeight(double value) {
    if (value != 0) setElevatorReference(getElevatorSetPoint() + (value * 0.10));
  }

  public void adjustWristHeight(double value) {
    if (value != 0) setWristReference(getWristSetPoint() + (value * 0.001));
  }
}
