// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Gyroscope extends SubsystemBase {
  /** Creates a new Gyroscope. */
  //Declare  gyro
  public static ADIS16470_IMU gyro = new ADIS16470_IMU();

  //Decalre final constants
  public static final double MaximumAllowedAngle = 2.5; 
  public static final double PreferedMaximumAngle = 2;

  //Declare variables and constants
  public static boolean balancePitchOnOff = false;
  public static boolean balanceRollOnOff = false;
  public static double XAxisInputValue;
  public static double YAxisInputValue;
  
  public Gyroscope() {}
  //check to see if needs to balance the pitch, x axis
  public static boolean autoBalancePitchModeOnOff(){
    if(gyro.getXComplementaryAngle() >= MaximumAllowedAngle){
      balancePitchOnOff = true;
    }
    else if(gyro.getXComplementaryAngle() <= PreferedMaximumAngle){
      balancePitchOnOff = false;
    }
    return true;
  }

  //checks to see if needs to balance the roll, y axis
  public static boolean autoBalanceRollModeOnOff(){
    if(gyro.getYComplementaryAngle() >= MaximumAllowedAngle){
      balanceRollOnOff = true;
    }
    else if(gyro.getYComplementaryAngle() <= PreferedMaximumAngle){
      balanceRollOnOff = false;
    }
    return true;
  }
  //returns the x rate value to be put into drive cartisian
  public static double getXRateValue(){
    autoBalancePitchModeOnOff();
    if(balancePitchOnOff == true){
      //want the speeds to be 0.18 to 0.3
      //code that used previously, does not adjust right, need something that is quadratic, exponetial rather than linear
      /*double pitchAngleRadians = gyro.getXComplementaryAngle() * (Math.PI / 180.0);
      XAxisInputValue = Math.sin(pitchAngleRadians) * -1;*/
    }
    else if(balancePitchOnOff == false){
      XAxisInputValue = RobotContainer.m_driverJoystick.getX();
    }
    return XAxisInputValue;
  }

  //returns the y rate value to be put into drive cartisian
  public static double getYRateValue(){
    autoBalanceRollModeOnOff();
    if(balanceRollOnOff == true){
      //want the speeds to be 0.18 to 0.3
      //code that used previously, does not adjust right, need something that is quadratic, exponetial rather than linear
      /*double rollAngleRadians = gyro.getYComplementaryAngle() * (Math.PI / 180.0);
      YAxisInputValue = Math.sin(rollAngleRadians) * -1;*/
    }
    else if(balanceRollOnOff == false){
      YAxisInputValue = RobotContainer.m_driverJoystick.getY();
    }
    return YAxisInputValue;
  }

    //Gyro code that does not work, tried to use the subsystems
    /*  
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
    */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
