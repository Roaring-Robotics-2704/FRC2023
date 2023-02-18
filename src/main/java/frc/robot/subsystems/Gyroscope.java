// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyroscope extends SubsystemBase {
  /** Creates a new Gyroscope. */
  //Declare  gyro
  public static ADIS16470_IMU gyro = new ADIS16470_IMU();

  // should be placed at center of robots axis of rotation
  // location does not really matter since measrure rotation not linear position but better to put at center
  // just need it to start out at a semi level position

  //Decalre final constants
  public static final double MaximumAllowedAngle = 2.5; 
  public static final double PreferedMaximumAngle = 2;

  //Declare variables and constants
  public static boolean balancePitchOnOff = false;
  public static boolean balanceRollOnOff = false;
  public static double XAxisInputValue;
  public static double YAxisInputValue;
  
  public Gyroscope() {}

  public static double zeroGyroX(double startAngle){
    double newAngle = 0;
      if(startAngle > 0){
        newAngle = gyro.getXComplementaryAngle() - startAngle;
      }
      else if(startAngle < 0){
        newAngle = gyro.getXComplementaryAngle() - startAngle;
      }
      else{
        newAngle = 0;
      }
    return newAngle;
  }

  public static double zeroGyroY(double startAngle){
    double newAngle = 0;
      if(startAngle > 0){
        newAngle = gyro.getYComplementaryAngle() - startAngle;
      }
      else if(startAngle < 0){
        newAngle = gyro.getYComplementaryAngle() - startAngle;
      }
      else{
        newAngle = 0;
      }
    return newAngle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
