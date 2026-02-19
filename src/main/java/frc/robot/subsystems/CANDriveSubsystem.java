// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

public class CANDriveSubsystem extends SubsystemBase {
  private final WPI_TalonSRX leftLeader;
  private final WPI_TalonSRX leftFollower;
  private final WPI_TalonSRX rightLeader;
  private final WPI_TalonSRX rightFollower;

  private final DifferentialDrive drive;

  public CANDriveSubsystem() {
    // create motor controllers for drive
    leftLeader = new WPI_TalonSRX(LEFT_LEADER_ID);
    leftFollower = new WPI_TalonSRX(LEFT_FOLLOWER_ID);
    rightLeader = new WPI_TalonSRX(RIGHT_LEADER_ID);
    rightFollower = new WPI_TalonSRX(RIGHT_FOLLOWER_ID);

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Reset controllers to a known state
    leftLeader.configFactoryDefault();
    leftFollower.configFactoryDefault();
    rightLeader.configFactoryDefault();
    rightFollower.configFactoryDefault();

    // Optional: set voltage compensation (helps consistent behavior across
    // battery voltages). Timeout of 250 ms used for configuration calls.
    leftLeader.configVoltageCompSaturation(12.0, 250);
    leftLeader.enableVoltageCompensation(true);
    rightLeader.configVoltageCompSaturation(12.0, 250);
    rightLeader.enableVoltageCompensation(true);
    leftFollower.configVoltageCompSaturation(12.0, 250);
    leftFollower.enableVoltageCompensation(true);
    rightFollower.configVoltageCompSaturation(12.0, 250);
    rightFollower.enableVoltageCompensation(true);

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

  @Override
  public void periodic() {
    // No-op
  }

  // Command factory to create command to drive the robot with joystick inputs.
  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.run(() -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()));
  }
}
