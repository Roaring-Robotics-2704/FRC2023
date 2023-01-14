// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int c_joystick = 0;
  }

  public static class DriveTrain{
    //ports
    //drive motors
    public static int c_frontRightMotor = 2;
    public static int c_frontLeftMotor = 1;
    public static int c_backRightMotor = 4;
    public static int c_backLeftMotor = 3;

    //joystick
    public static int c_joystickMain = 0;
    public static int c_joystickButton = 1;
  }
  public static class GyroConstants{
    public static final double c_MaximumAllowedAngle = 2.5; //is the maximum allowed in order to be consider engaged and docked(level)
    public static final double c_TargetAngle  = 2; //this number might need to be changed, it is when we stop adjusting

  }
}
