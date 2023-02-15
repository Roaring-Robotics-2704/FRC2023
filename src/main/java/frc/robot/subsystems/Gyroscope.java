
package frc.robot.subsystems;

import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
  public static boolean autoBalanceRollModeOnOff(){
    if(gyro.getYComplementaryAngle() >= MaximumAllowedAngle){
      balanceRollOnOff = true;
    }
    else if(gyro.getYComplementaryAngle() <= PreferedMaximumAngle){
      balanceRollOnOff = false;
    }
    return true;
  }
  public static boolean autoBalancePitchModeOnOff(){
    if(gyro.getXComplementaryAngle() >= MaximumAllowedAngle){
      balancePitchOnOff = true;
    }
    else if(gyro.getXComplementaryAngle() <= PreferedMaximumAngle){
      balancePitchOnOff = false;
    }
    return true;
  }

  public static double getYRateValue(){
    autoBalanceRollModeOnOff();
    if(balanceRollOnOff == true){
      //want the speeds to be 0.18 to 0.3
      //code that used previously, does not adjust right, need something that is quadratic, exponetial rather than linear
      //double rollAngleRadians = gyro.getYComplementaryAngle() * (Math.PI / 180.0);
      //YAxisInputValue = Math.sin(rollAngleRadians) * -1;
    }
    else if(balanceRollOnOff == false){
      YAxisInputValue = 0;
    }
    return YAxisInputValue;
  }
  public static double getXRateValue(){
    autoBalancePitchModeOnOff();
    if(balancePitchOnOff == true){
      //want the speeds to be 0.18 to 0.3
      //code that used previously, does not adjust right, need something that is quadratic, exponetial rather than linear
      //double pitchAngleRadians = gyro.getXComplementaryAngle() * (Math.PI / 180.0);
      //XAxisInputValue = Math.sin(pitchAngleRadians) * -1;
    }
    else if(balancePitchOnOff == false){
      XAxisInputValue = 0;
    }
    return XAxisInputValue;
  }


}