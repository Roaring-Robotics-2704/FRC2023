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
    public static final int kDriverControllerPort = 0;
  }


  public static int c_frontleftDriveMotor = 1;
  public static int c_frontrightDriveMotor = 2;
  public static int c_backleftDriveMotor = 3;
  public static int c_backrightDriveMotor = 4;

  public static int c_leftJoystickAxisx = 1;
  public static int c_rightJoystickAxisx = 5;
  public static int c_rightJoystickAxisy = 5;
  public static int c_lightLength = 100;
  
  public static int c_ledPort = 0;
  public static int[] yellowRGB = {255/2, 150/2, 0/2};
  public static int[] blueRGB = {0,100/2,255/2};
  public static int[] MiddleSchoolOrangeRGB = {255,25,0}; //Matthew's favorite color

  public static double c_speedcap = 0;//0.75 speed is good. y of xbox times this number.
}
