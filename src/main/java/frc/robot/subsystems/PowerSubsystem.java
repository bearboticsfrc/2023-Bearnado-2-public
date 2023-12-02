// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

/** Monitor and log the power distribution hub current usage. */
public class PowerSubsystem extends SubsystemBase {
  private final PowerDistribution powerDistribution;

  private DoubleLogEntry totalCurrent;

  /** Creates a new PowerSubsystem. */
  public PowerSubsystem() {
    powerDistribution = new PowerDistribution(1, ModuleType.kRev);

    DataLog log = DataLogManager.getLog();
    totalCurrent = new DoubleLogEntry(log, "/power/total_current");

    ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

    compTab.addNumber("Total current", this::getTotalCurrent).withPosition(6, 0);
    compTab.addNumber("Voltage", this::getVoltage).withPosition(6, 1);
  }

  /**
   * Returns the total current from the PDP.
   *
   * @return The total current.
   */
  public double getTotalCurrent() {
    return powerDistribution.getTotalCurrent();
  }

  /**
   * Returns the input voltage retreived from the PDP.
   *
   * @return The voltage.
   */
  public double getVoltage() {
    return powerDistribution.getVoltage();
  }

  @Override
  public void periodic() {
    if (Robot.isRobotEnabled()) {
      totalCurrent.append(powerDistribution.getVoltage());
    }
  }
}
