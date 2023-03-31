package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Gyroscope extends SubsystemBase {
  /** Creates a new Gyroscope. */
  //Declare  gyro
  public static ADIS16470_IMU gyro = RobotContainer.m_imu;

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