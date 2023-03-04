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
public class DriveRobot extends CommandBase {
  /** Creates a new DriveRobot. */
  public DriveRobot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Drivetrain);
  }

  PIDController align = new PIDController(Constants.zpid.p,Constants.zpid.i,Constants.zpid.d);
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

  public Boolean mode;
  private double angle;
  public double turbo;
  public double precision;
  public double turboamount;
  double joystickxz; // getRawAxis(Constants.c_leftJoystickAxisx);
  double joystickyz;
  double joystickx; // getRawAxis(Constants.c_rightJoystickAxisx);
  double joysticky;

  @Override
  public void execute() {
    
    turbo = RobotContainer.xbox.getRightTriggerAxis();
    precision = RobotContainer.xbox.getLeftTriggerAxis()/2;
  
      turboamount = turbo-precision+Constants.c_speedcap;

    
    SmartDashboard.putNumber("turbo amount", turboamount);
    SmartDashboard.putNumber("turbo", turbo);
    if (RobotContainer.Drivescheme.getSelected()) {
       joystickxz = RobotContainer.xbox.getLeftX(); // getRawAxis(Constants.c_leftJoystickAxisx);
       joystickyz = RobotContainer.xbox.getLeftY();
       joystickx = RobotContainer.xbox.getRightX(); // getRawAxis(Constants.c_rightJoystickAxisx);
       joysticky = -RobotContainer.xbox.getRightY();
    }
    else {
     joystickxz = RobotContainer.xbox.getRightX(); // getRawAxis(Constants.c_leftJoystickAxisx);
     joystickyz = RobotContainer.xbox.getRightY();
     joystickx = RobotContainer.xbox.getLeftX(); // getRawAxis(Constants.c_rightJoystickAxisx);
     joysticky = -RobotContainer.xbox.getLeftY(); // getRawAxis(Constants.c_rightJoystickAxisy);
    }
    double outputx = joystickx * turboamount;
    double outputy = joysticky * turboamount;
    double outputz = joystickxz * turboamount;
    mode = RobotContainer.DriveMode.getSelected();
    
    SmartDashboard.putNumber("x", outputx);
    SmartDashboard.putNumber("y", outputy);
    SmartDashboard.putNumber("z", outputz);
    SmartDashboard.putNumber("vector angle",vector(joystickxz,joystickyz));
    SmartDashboard.putNumber("output heading", angle);
    SmartDashboard.putNumber("actual heading", -RobotContainer.m_imu.getAngle());;
    if (mode) {
      RobotContainer.m_Drivetrain.driveCartesian(outputy,outputx,outputz,-gyro.getAngle());   
    }
    else {
      RobotContainer.m_Drivetrain.driveCartesian(outputy, outputx,outputz, 0);
    }
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
