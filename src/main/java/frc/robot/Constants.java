// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.SIM; // TODO: change this if sim
  // public static final boolean isSYSID = true; // TODO: change this if sysid
  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY_REAL,

    REPLAY_SIM,
  }
  
  public static class OperatorConstants {
    public static final int kDRIVER_CONTROLLER_PORT = 0;

    // Joystick Deadband
    // yes, this high
    public static final double LEFT_X_DEADBAND = 0.09;
    public static final double LEFT_Y_DEADBAND = 0.09;
    
    public static final double RIGHT_X_DEADBAND = 0.09;
    
  }

  public static final class Auton
  {

    public static final PIDConstants TRANSLATION_PID_CONFIG = new PIDConstants(0, 0, 0.0000);
    public static final PIDConstants ANGLE_PID_CONFIG = new PIDConstants(0, 0, 0.00008); 

    public static final double MAX_SPEED = 4;
    public static final double MAX_ACCELERATION = 2;
    public static final double MAX_ANGULAR_VELO_RPS = 2; // 1
    public static final double MAX_ANGULAR_ACCEL_RPS_SQUARED = 1; // 0.5

    public static final HolonomicPathFollowerConfig DRIVE_CONTROLLER_CONFIG = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        Constants.Auton.TRANSLATION_PID_CONFIG, // Translation PID constants
                        Constants.Auton.ANGLE_PID_CONFIG, // Rotation PID constants
                        Constants.Auton.MAX_SPEED, // Max module speed, in m/s
                        // I LOVE THE PYTHAGPREAN THEOREM
                        0.5384061785684372, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
    );

  }

  public static class DrivetrainConstants {
    public static final double kMAX_SPEED_METERS_PER_SECOND = 4;
    public static final double kMAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2;
    public static final double kMAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI;
    public static final double kMAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2 * Math.PI;


    // x, y from center of the robot
    public static final Translation2d kFRONT_RIGHT_POSITION_METERS = new Translation2d(0.38, -0.38);
    public static final Translation2d kBACK_RIGHT_POSITION_METERS = new Translation2d(-0.38, -0.38);
    public static final Translation2d kFRONT_LEFT_POSITION_METERS = new Translation2d(0.38, 0.38);
    public static final Translation2d kBACK_LEFT_POSITION_METERS = new Translation2d(-0.38, 0.38);

    public static final double kFRONT_RIGHT_ANGLE_OFFSET_DEG = 20;
    public static final double kBACK_RIGHT_ANGLE_OFFSET_DEG = 20;
    public static final double kFRONT_LEFT_ANGLE_OFFSET_DEG = 20;
    public static final double kBACK_LEFT_ANGLE_OFFSET_DEG = 20;

    public static final double kDRIVE_MOTOR_TO_OUTPUT_SHAFT_RATIO = .23;
    public static final double kANGLE_MOTOR_TO_OUTPUT_SHAFT_RATIO = .23;

    public static final double kDRIVE_WHEEL_RADIUS_METERS = .1;

    public static final double kMAX_ANGLE_VOLTAGE = 12;
    public static final double kMAX_DRIVE_VOLTAGE = 12;


  }
}
