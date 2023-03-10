// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Mechanum.mechanumConf.anglepid;

public class DriveRobot extends CommandBase {
  /** Creates a new DriveRobot. */
  public DriveRobot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Drivetrain);
  }

  ADIS16470_IMU gyro = RobotContainer.m_imu;
  public static double vector(double x, double y) {
      double angleRadians = Math.atan2(y, x);
      double angleDegrees = Math.toDegrees(angleRadians);
      return angleDegrees;
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_imu.reset();
    RobotContainer.m_imu.calibrate();
    RobotContainer.m_imu.reset();
    
  }
  public static double scale_with_sign(double axis_value) {
    final double SCALE_EXPONENT = 1;
    final double DEADZONE_SIZE = Constants.deadzone;

    // creates scale factor based on deadzone size, so entire output area is still
    // usable
    double ScaleFactor = 1 / (1 - DEADZONE_SIZE);

    // x/|x| = 1 or -1 depending on sign of x
    double axis_sign = axis_value / Math.abs(axis_value);

    double unsigned_return_value = Math.abs(axis_value);

    unsigned_return_value = unsigned_return_value - DEADZONE_SIZE;

    if (unsigned_return_value < 0) { // set value to 0 if it's within the DEADZONE
      unsigned_return_value = 0;
    }
    unsigned_return_value = unsigned_return_value * ScaleFactor;

    unsigned_return_value = Math.pow(unsigned_return_value, SCALE_EXPONENT);

    return axis_sign * unsigned_return_value;
  }

  public Boolean mode;
  private double angle;
  public double turbo;
  public double precision;
  public double turboamount;
  double joystickxz; // getRawAxis(Constants.c_leftJoystickAxisx);
  double joystickyz;
  double joystickx; // getRawAxis(Constants.c_rightJoystickAxisx);
  double joysticky;
  double correctangle = 0;
  PIDController anglePID = new PIDController(anglepid.p, anglepid.i, anglepid.d);

  @Override
  public void execute() {
    
    turbo = RobotContainer.xbox.getRightTriggerAxis();
    precision = RobotContainer.xbox.getLeftTriggerAxis();
  
      turboamount = turbo-precision+Constants.c_speedcap;

    
    SmartDashboard.putNumber("turbo amount", turboamount);
    SmartDashboard.putNumber("turbo", turbo);
    if (RobotContainer.Drivescheme.getSelected()) {
       joystickxz = RobotContainer.xbox.getLeftX(); // getRawAxis(Constants.c_leftJoystickAxisx);
       joystickx = RobotContainer.xbox.getRightX(); // getRawAxis(Constants.c_rightJoystickAxisx);
       joysticky = -RobotContainer.xbox.getRightY();
    }
    else {
     joystickxz = RobotContainer.xbox.getRightX(); // getRawAxis(Constants.c_leftJoystickAxisx);
     joystickx = RobotContainer.xbox.getLeftX(); // getRawAxis(Constants.c_rightJoystickAxisx);
     joysticky = -RobotContainer.xbox.getLeftY(); // getRawAxis(Constants.c_rightJoystickAxisy);
    }
    double outputx = joystickx * turboamount;
    double outputy = joysticky * turboamount;
    double outputz = joystickxz * turboamount;
    outputz = scale_with_sign(outputz);
    correctangle = correctangle+outputz;

    SmartDashboard.putNumber("x", outputx);
    SmartDashboard.putNumber("y", outputy);
    SmartDashboard.putNumber("z", outputz);
    SmartDashboard.putNumber("vector angle",vector(joystickxz,joystickyz));
    SmartDashboard.putNumber("output heading", angle); 
    SmartDashboard.putNumber("actual heading", -RobotContainer.m_imu.getAngle());;
   
    RobotContainer.m_Drivetrain.driveCartesian(outputy, outputx,anglePID.calculate(-RobotContainer.m_imu.getAngle(), correctangle));
    if (RobotContainer.xbox.getLeftBumper()) {
      RobotContainer.m_imu.reset();
      RobotContainer.xbox.setRumble(RumbleType.kBothRumble, 0.5);
    }
    else {
      RobotContainer.xbox.setRumble(RumbleType.kBothRumble, 0);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
