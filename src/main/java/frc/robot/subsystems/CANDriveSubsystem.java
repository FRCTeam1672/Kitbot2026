// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

public class CANDriveSubsystem extends SubsystemBase {
  private final WPI_TalonSRX leftLeader;
  private final WPI_VictorSPX leftFollower;
  private final WPI_TalonSRX rightLeader;
  private final WPI_VictorSPX rightFollower;

  private final DifferentialDrive drive;

  public CANDriveSubsystem() {
    // Create motor controllers
    leftLeader = new WPI_TalonSRX(LEFT_LEADER_ID);
    leftFollower = new WPI_VictorSPX(LEFT_FOLLOWER_ID);
    rightLeader = new WPI_TalonSRX(RIGHT_LEADER_ID);
    rightFollower = new WPI_VictorSPX(RIGHT_FOLLOWER_ID);

    // Coast mode on all drive motors
    leftLeader.setNeutralMode(NeutralMode.Coast);
    leftFollower.setNeutralMode(NeutralMode.Coast);
    rightLeader.setNeutralMode(NeutralMode.Coast);
    rightFollower.setNeutralMode(NeutralMode.Coast);

    // Set up differential drive with Talon SRX controllers
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Configure followers to follow their leaders
    leftFollower.follow(leftLeader);
    leftFollower.setInverted(InvertType.FollowMaster);
    rightFollower.follow(rightLeader);
    rightFollower.setInverted(InvertType.FollowMaster);

    // Invert left side so that positive values drive both sides forward
    leftLeader.setInverted(true);
  }

  @Override
  public void periodic() {
  }

  // Command factory to create command to drive the robot with joystick inputs.
  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.run(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()));
  }
}
