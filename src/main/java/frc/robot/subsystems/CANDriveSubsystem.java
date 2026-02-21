// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

public class CANDriveSubsystem extends SubsystemBase {
  private final WPI_TalonSRX leftLeader;
  private final WPI_TalonSRX leftFollower;
  private final WPI_TalonSRX rightLeader;
  private final WPI_TalonSRX rightFollower;

  public CANDriveSubsystem() {
    // create motor controllers for drive
    leftLeader = new WPI_TalonSRX(LEFT_LEADER_ID);
    leftFollower = new WPI_TalonSRX(LEFT_FOLLOWER_ID);
    rightLeader = new WPI_TalonSRX(RIGHT_LEADER_ID);
    rightFollower = new WPI_TalonSRX(RIGHT_FOLLOWER_ID);

    // Reset controllers to a known state
    leftLeader.configFactoryDefault();
    leftFollower.configFactoryDefault();
    rightLeader.configFactoryDefault();
    rightFollower.configFactoryDefault();

    // Configure voltage compensation for all motors
    configureVoltageCompensation(leftLeader);
    configureVoltageCompensation(leftFollower);
    configureVoltageCompensation(rightLeader);
    configureVoltageCompensation(rightFollower);

    // Configure followers
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    // Set neutral mode and inversion so positive speeds drive the robot forward.
    leftLeader.setNeutralMode(NeutralMode.Brake);
    rightLeader.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    // Invert the left side so forward is positive on both sides
    leftLeader.setInverted(true);
    leftFollower.setInverted(true);
    rightLeader.setInverted(false);
    rightFollower.setInverted(false);
  }

  /**
   * Configures voltage compensation for a motor controller.
   * 
   * @param motor the motor controller to configure
   */
  private void configureVoltageCompensation(WPI_TalonSRX motor) {
    motor.configVoltageCompSaturation(12.0, 250);
    motor.enableVoltageCompensation(true);
  }

  @Override
  public void periodic() {
    // Log drivetrain motor outputs for debugging and tuning
    SmartDashboard.putNumber(LEFTSIDE_OUTPUT_KEY, leftLeader.get());
    SmartDashboard.putNumber(RIGHTSIDE_OUTPUT_KEY, rightLeader.get());
  }

  /**
   * Creates a command to drive the robot using arcade drive with joystick input.
   * Applies deadzone filtering to prevent drift and power scaling for easier control.
   * 
   * @param xSpeed supplier providing forward/backward speed (-1.0 to 1.0)
   * @param zRotation supplier providing rotation speed (-1.0 to 1.0)
   * @return a command that drives the robot in arcade mode
   */
  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.run(() -> {
      double xSpeedValue = applyDeadzone(xSpeed.getAsDouble());
      double zRotationValue = applyDeadzone(zRotation.getAsDouble());
      
      // Apply right side power scaling to compensate for mechanical imbalance
      rightLeader.set(limitOutput((xSpeedValue + zRotationValue) * RIGHT_SIDE_POWER_SCALING));
      leftLeader.set(limitOutput(xSpeedValue - zRotationValue));
    });
  }

  /**
   * Applies deadzone filtering to joystick input to prevent drift.
   * Values within the deadzone threshold are treated as zero.
   * 
   * @param value the raw joystick value
   * @return the deadzone-filtered value
   */
  private double applyDeadzone(double value) {
    return Math.abs(value) < JOYSTICK_DEADZONE ? 0.0 : value;
  }

  private double limitOutput(double value) {
    return Math.max(-1.0, Math.min(1.0, value));
  }
}
