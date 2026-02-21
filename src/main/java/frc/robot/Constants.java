// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 21;
    public static final int LEFT_FOLLOWER_ID = 11;
    public static final int RIGHT_LEADER_ID = 22;
    public static final int RIGHT_FOLLOWER_ID = 12;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;

    // Power scaling factor for right side to compensate for mechanical imbalance.
    // Adjust this value between 0.0 and 1.0 to reduce right side power.
    // Start with 0.9 and adjust based on robot behavior.
    public static final double RIGHT_SIDE_POWER_SCALING = 0.9;

    // Joystick deadzone to prevent drift. Values below this threshold are treated as zero.
    // Typical range: 0.02 to 0.1. Start with 0.05 and adjust based on controller sensitivity.
    public static final double JOYSTICK_DEADZONE = 0.05;

    // Telemetry keys for SmartDashboard logging
    public static final String LEFTSIDE_OUTPUT_KEY = "Drive/Left Output";
    public static final String RIGHTSIDE_OUTPUT_KEY = "Drive/Right Output";
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int FEEDER_MOTOR_ID = 6;
    public static final int INTAKE_LAUNCHER_MOTOR_ID = 5;

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information.
    // Intaking: positive values spin motors to draw fuel in. Typical range: 5-6V.
    public static final double INTAKING_FEEDER_VOLTAGE = -5;
    public static final double INTAKING_INTAKE_VOLTAGE = 6;
    
    // Launching: high power spinup and launch. Feeder reverses to push fuel. Range: 10-12V.
    public static final double LAUNCHING_FEEDER_VOLTAGE = 11;
    public static final double LAUNCHING_LAUNCHER_VOLTAGE = 12;
    public static final double SPIN_UP_FEEDER_VOLTAGE = -10;
    public static final double SPIN_UP_SECONDS = 1;

    // Low power launch voltage values for reduced power launches. Range: 9-11V.
    public static final double LOW_POWER_LAUNCHING_FEEDER_VOLTAGE = 10;
    public static final double LOW_POWER_LAUNCHING_LAUNCHER_VOLTAGE = 11;
    public static final double LOW_POWER_SPIN_UP_FEEDER_VOLTAGE = -9;
    public static final double LOW_POWER_SPIN_UP_SECONDS = 0.5;

    // SmartDashboard keys for fuel mechanism tuning
    public static final String INTAKING_FEEDER_KEY = "Fuel/Intaking feeder roller value";
    public static final String INTAKING_INTAKE_KEY = "Fuel/Intaking intake roller value";
    public static final String LAUNCHING_FEEDER_KEY = "Fuel/Launching feeder roller value";
    public static final String LAUNCHING_LAUNCHER_KEY = "Fuel/Launching launcher roller value";
    public static final String SPINUP_FEEDER_KEY = "Fuel/Spin-up feeder roller value";
    public static final String LOW_POWER_LAUNCHING_FEEDER_KEY = "Fuel/Low Power Launching feeder roller value";
    public static final String LOW_POWER_LAUNCHING_LAUNCHER_KEY = "Fuel/Low Power Launching launcher roller value";
    public static final String LOW_POWER_SPINUP_FEEDER_KEY = "Fuel/Low Power Spin-up feeder roller value";
  }

  public static final class DriverConstants {
    // Port constants for driver controller. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;

    // This value is multiplied by the joystick value when driving the robot to
    // help avoid driving and turning too fast and being difficult to control.
    // Typical range: 0.8 to 0.9.
    public static final double DRIVE_SCALING = 0.9;
    public static final double ROTATION_SCALING = 0.8;
  }
}
