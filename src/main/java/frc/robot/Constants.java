// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static int kleftDriveCanID = 2;
    public static int krightDriveCanID = 1;

    public static double cameraHeightMeters = Units.inchesToMeters(14.25);
    public static double targetHeightMeters = Units.inchesToMeters(21);
    public static double cameraPitchRadians = Units.degreesToRadians(20);

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

  }
}
