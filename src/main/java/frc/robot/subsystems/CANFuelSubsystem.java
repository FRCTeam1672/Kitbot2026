// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller;

  /** Creates a new CANFuelSubsystem. */
  @SuppressWarnings("removal")
  public CANFuelSubsystem() {
    // Create brushed motors for each of the motors on the launcher mechanism
    intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushed);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushed);

    // Configure the feeder roller with current limit and apply config
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure the launcher roller with inverted direction and current limit
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(true);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Put default values for various fuel operations onto the SmartDashboard.
    // Methods in this subsystem read these values at runtime so you can tune
    // voltages without recompiling. When satisfied, replace the values in
    // `Constants.java` with the tuned numbers.
    SmartDashboard.putNumber(INTAKING_FEEDER_KEY, INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber(INTAKING_INTAKE_KEY, INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber(LAUNCHING_FEEDER_KEY, LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber(LAUNCHING_LAUNCHER_KEY, LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber(SPINUP_FEEDER_KEY, SPIN_UP_FEEDER_VOLTAGE);
    SmartDashboard.putNumber(LOW_POWER_LAUNCHING_FEEDER_KEY, LOW_POWER_LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber(LOW_POWER_LAUNCHING_LAUNCHER_KEY, LOW_POWER_LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber(LOW_POWER_SPINUP_FEEDER_KEY, LOW_POWER_SPIN_UP_FEEDER_VOLTAGE);
  }

  /**
   * Sets the fuel intake system to intake fuel. Both feeder and intake motors
   * operate at speeds configured in SmartDashboard or Constants.
   */
  public void intake() {
    feederRoller.setVoltage(SmartDashboard.getNumber(INTAKING_FEEDER_KEY, INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(SmartDashboard.getNumber(INTAKING_INTAKE_KEY, INTAKING_INTAKE_VOLTAGE));
  }

  /**
   * Ejects fuel back out through the intake. Uses negated intake voltages
   * to reverse the motor direction.
   */
  public void eject() {
    feederRoller.setVoltage(-SmartDashboard.getNumber(INTAKING_FEEDER_KEY, INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(-SmartDashboard.getNumber(INTAKING_INTAKE_KEY, INTAKING_INTAKE_VOLTAGE));
  }

  /**
   * Launches fuel at full power. Both feeder and launcher motors run at full
   * voltages configured in Constants.
   */
  public void launch() {
    feederRoller.setVoltage(SmartDashboard.getNumber(LAUNCHING_FEEDER_KEY, LAUNCHING_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(SmartDashboard.getNumber(LAUNCHING_LAUNCHER_KEY, LAUNCHING_LAUNCHER_VOLTAGE));
  }

  /**
   * Stops both rollers immediately.
   */
  public void stop() {
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
  }

  /**
   * Spins up the launcher at full power while pushing fuel toward the launcher.
   * Feeder runs in reverse at reduced voltage. Should be followed by launch().
   */
  public void spinUp() {
    feederRoller.setVoltage(SmartDashboard.getNumber(SPINUP_FEEDER_KEY, SPIN_UP_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(SmartDashboard.getNumber(LAUNCHING_LAUNCHER_KEY, LAUNCHING_LAUNCHER_VOLTAGE));
  }

  /**
   * Creates a command that runs spinUp() while scheduled.
   * @return a spinup command
   */
  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  /**
   * Creates a command that runs launch() while scheduled.
   * @return a launch command
   */
  public Command launchCommand() {
    return this.run(() -> launch());
  }

  /**
   * Launches fuel at reduced power for shorter-range or gentler launches.
   * Lower voltages reduce stress on motors and conserve energy.
   */
  public void lowPowerLaunch() {
    feederRoller.setVoltage(SmartDashboard.getNumber(LOW_POWER_LAUNCHING_FEEDER_KEY, LOW_POWER_LAUNCHING_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(SmartDashboard.getNumber(LOW_POWER_LAUNCHING_LAUNCHER_KEY, LOW_POWER_LAUNCHING_LAUNCHER_VOLTAGE));
  }

  /**
   * Spins up the launcher at low power while pushing fuel toward the launcher.
   * Used before lowPowerLaunch() for reduced-power launching.
   */
  public void lowPowerSpinUp() {
    feederRoller.setVoltage(SmartDashboard.getNumber(LOW_POWER_SPINUP_FEEDER_KEY, LOW_POWER_SPIN_UP_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(SmartDashboard.getNumber(LOW_POWER_LAUNCHING_LAUNCHER_KEY, LOW_POWER_LAUNCHING_LAUNCHER_VOLTAGE));
  }

  /**
   * Creates a command that runs lowPowerSpinUp() while scheduled.
   * @return a low-power spinup command
   */
  public Command lowPowerSpinUpCommand() {
    return this.run(() -> lowPowerSpinUp());
  }

  /**
   * Creates a command that runs lowPowerLaunch() while scheduled.
   * @return a low-power launch command
   */
  public Command lowPowerLaunchCommand() {
    return this.run(() -> lowPowerLaunch());
  }
}
