// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

public class CANDriveSubsystem extends SubsystemBase {
  private static final double VOLTAGE_SATURATION = 12.0;
  private static final int CONFIG_TIMEOUT_MS = 250;

  private final WPI_TalonSRX leftLeader;
  private final WPI_VictorSPX leftFollower;
  private final WPI_TalonSRX rightLeader;
  private final WPI_VictorSPX rightFollower;

  private final DifferentialDrive drive;

  public CANDriveSubsystem() {
    // Create motor controllers for drive
    leftLeader = new WPI_TalonSRX(LEFT_LEADER_ID);
    leftFollower = new WPI_VictorSPX(LEFT_FOLLOWER_ID);
    rightLeader = new WPI_TalonSRX(RIGHT_LEADER_ID);
    rightFollower = new WPI_VictorSPX(RIGHT_FOLLOWER_ID);

    // Reset controllers to a known state
    resetMotors();

    // Configure voltage compensation for consistent behavior across battery voltages
    configureVoltageCompensation();

    // Configure followers to follow leaders
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    // Configure neutral mode and motor inversion for forward motion
    configureMotorProperties();

    // Set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);
  }

  /**
   * Reset all motor controllers to factory defaults.
   */
  private void resetMotors() {
    leftLeader.configFactoryDefault(CONFIG_TIMEOUT_MS);
    leftFollower.configFactoryDefault(CONFIG_TIMEOUT_MS);
    rightLeader.configFactoryDefault(CONFIG_TIMEOUT_MS);
    rightFollower.configFactoryDefault(CONFIG_TIMEOUT_MS);
  }

  /**
   * Configure voltage compensation for all motor controllers to improve
   * consistency across varying battery voltages.
   */
  private void configureVoltageCompensation() {
    leftLeader.configVoltageCompSaturation(VOLTAGE_SATURATION, CONFIG_TIMEOUT_MS);
    leftLeader.enableVoltageCompensation(true);
    rightLeader.configVoltageCompSaturation(VOLTAGE_SATURATION, CONFIG_TIMEOUT_MS);
    rightLeader.enableVoltageCompensation(true);
    leftFollower.configVoltageCompSaturation(VOLTAGE_SATURATION, CONFIG_TIMEOUT_MS);
    leftFollower.enableVoltageCompensation(true);
    rightFollower.configVoltageCompSaturation(VOLTAGE_SATURATION, CONFIG_TIMEOUT_MS);
    rightFollower.enableVoltageCompensation(true);
  }

  /**
   * Configure neutral mode and motor inversion.
   * Left side is inverted so positive speeds drive the robot forward on both sides.
   */
  private void configureMotorProperties() {
    leftLeader.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightLeader.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    leftLeader.setInverted(true);
    leftFollower.setInverted(true);
    rightLeader.setInverted(false);
    rightFollower.setInverted(false);
  }

  /**
   * Command factory to create a drive command with joystick inputs.
   * Applies right side power scaling to compensate for mechanical imbalance.
   * 
   * @param xSpeed forward/backward speed supplier
   * @param zRotation rotation speed supplier
   * @return a drive command
   */
  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.run(() -> {
      double speed = xSpeed.getAsDouble();
      double rotation = zRotation.getAsDouble();
      drive.arcadeDrive(speed, rotation);
      
      // Apply right side power scaling for mechanical balance
      rightLeader.set(rightLeader.get() * RIGHT_SIDE_POWER_SCALING);
      rightFollower.set(rightFollower.get() * RIGHT_SIDE_POWER_SCALING);
    });
  }
}
