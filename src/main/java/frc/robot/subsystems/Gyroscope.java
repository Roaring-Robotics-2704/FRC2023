// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gyroscope extends SubsystemBase {
  /** Creates a new Gyroscope. */
  public static ADIS16470_IMU gyro = new ADIS16470_IMU();
  

  public Gyroscope() {}

    static boolean autoBalancePitchMode = false; //same as X
    static boolean autoBalanceRollMode = false; //same as Y

    static final double MaximumAllowedAngle = 2.5; //is the maximum allowed in order to be consider engaged and docked(level)
    static final double TargetAngle  = 2; //this number might need to be changed, it is when we stop adjusting

    public static double pitchAngleDegrees = gyro.getXComplementaryAngle();
    public static double rollAngleDegrees = gyro.getYComplementaryAngle();

    public static boolean Pitch(){
      
      if(gyro.getXComplementaryAngle() >= 5){
        System.out.print("inside pitch");
        autoBalancePitchMode = true;
      }
      return autoBalancePitchMode;
    }

    public void level(){
      System.out.println("has run level");
      if ( !autoBalancePitchMode && (Math.abs(pitchAngleDegrees) >= Math.abs(Constants.GyroConstants.c_MaximumAllowedAngle))) {
        autoBalancePitchMode = true;
      }
      else if ( autoBalancePitchMode && (Math.abs(pitchAngleDegrees) <= Math.abs(Constants.GyroConstants.c_TargetAngle))) {
        autoBalancePitchMode = false;
      }
      if ( !autoBalanceRollMode && (Math.abs(rollAngleDegrees) >= Math.abs(Constants.GyroConstants.c_MaximumAllowedAngle))) {
        autoBalanceRollMode = true;
      }
      else if ( autoBalanceRollMode && (Math.abs(rollAngleDegrees) <= Math.abs(Constants.GyroConstants.c_TargetAngle))) {
        autoBalanceRollMode = false;
      }
    }

    public static boolean PitchMode(){
      System.out.println("pitchmode");
      if ( !autoBalancePitchMode && (Math.abs(pitchAngleDegrees) >= Math.abs(Constants.GyroConstants.c_MaximumAllowedAngle))) {
        System.out.println("autobalnce pitch if");
        autoBalancePitchMode = true;
      }
      else if ( autoBalancePitchMode && (Math.abs(pitchAngleDegrees) <= Math.abs(Constants.GyroConstants.c_TargetAngle))) {
        autoBalancePitchMode = false;
      }
      if(true){
        autoBalancePitchMode = true;
      }
      return autoBalancePitchMode;
    }

    public static boolean getIfPitchModeOn(){
      return autoBalancePitchMode;
    }

    public static boolean getIfRollModeOn(){
      return autoBalanceRollMode;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
