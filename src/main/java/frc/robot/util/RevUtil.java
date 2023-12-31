// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;

/** Convenience methods for using the RevRobotics api. */
public class RevUtil {

  /**
   * Accepts a command that returns a REVLibError and if it is not "ok" then print an error to the
   * driver station
   *
   * @param error A REVLibError from any REV API command
   * @param message string message to display of the error is not ok
   */
  public static void checkRevError(REVLibError error, String message) {
    if (error != REVLibError.kOk) {
      DriverStation.reportError(String.format("%s: %s", message, error.toString()), false);
    }
  }

  /**
   * Sets the Status 0,1,2,3 frame rates for a CANSparkMax motor controller
   *
   * @param motor A CANSparkMax motor controller
   * @param desc Description of the motor ("Climber1", "Left Drive", etc)
   */
  public static void setPeriodicFramePeriodLow(CANSparkMax motor, String desc) {
    checkRevError(
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20),
        "Failed to set periodic status frame 0 rate for " + desc);
    checkRevError(
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20),
        "Failed to set periodic status frame 1 rate for " + desc);
    checkRevError(
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 100),
        "Failed to set periodic status frame 2 rate for " + desc);
    checkRevError(
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 100),
        "Failed to set periodic status frame 3 rate for " + desc);
  }

  /**
   * Sets the Status 0,1,2,3 frame rates for a CANSparkMax motor controller
   *
   * @param motor A CANSparkMax motor controller
   * @param desc Description of the motor ("Climber1", "Left Drive", etc)
   */
  public static void setPeriodicFramePeriodHigh(CANSparkMax motor, String desc) {
    checkRevError(
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10),
        "Failed to set periodic status frame 0 rate for " + desc);
    checkRevError(
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20),
        "Failed to set periodic status frame 1 rate for " + desc);
    checkRevError(
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20),
        "Failed to set periodic status frame 2 rate for " + desc);
    checkRevError(
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 50),
        "Failed to set periodic status frame 3 rate for " + desc);
  }
}
