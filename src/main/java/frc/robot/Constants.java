// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY_REAL,

    REPLAY_SIM
  }
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class DrivetrainConstants {
    // x, y from center of the robot

    public static final Translation2d frontRightPositionMeters = new Translation2d(1, 1);
    public static final Translation2d backRightPositionMeters = new Translation2d(1, -1);
    public static final Translation2d frontLeftPositionMeters = new Translation2d(-1, 1);
    public static final Translation2d backLeftPositionMeters = new Translation2d(-1, -1);

    public static final double frontRightAngleOffsetDeg = 20;
    public static final double backRightAngleOffsetDeg = 20;
    public static final double frontLeftAngleOffsetDeg = 20;
    public static final double backLeftAngleOffsetDeg = 20;

    public static final double driveMotorOutputToShaftRatio = .23;
    public static final double angleMotorOutputToShaftRatio = .23;

    public static final double driveWheelRadiusMeters = .1;


  }
}
