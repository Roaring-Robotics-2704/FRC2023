// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  public Drivetrain() { }
  
  WPI_VictorSPX leftDriveMotor = new WPI_VictorSPX(Constants.kleftDriveCanID);
  WPI_VictorSPX rightDriveMotor = new WPI_VictorSPX(Constants.krightDriveCanID);
  //1.45 does work mostly
  PIDController distancePID = new PIDController(Constants.distance_kP, Constants.distance_kI, Constants.distance_kD);
  //.05 does work but is being tuned to be better
  PIDController rotationPID = new PIDController(Constants.rotation_kP, Constants.rotation_kI, Constants.rotation_kD);

  DifferentialDrive m_drive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void driveRobot(double forwardSpeed, double rotationSpeed) {
    m_drive.arcadeDrive(forwardSpeed, rotationSpeed);
  }

  public void distanceDrivingPID(double measurement, double setpoint) {
    double forwardSpeed = distancePID.calculate(measurement, setpoint);
    leftDriveMotor.set(forwardSpeed);
    rightDriveMotor.set(forwardSpeed);
  }

  public void rotationDrivingPID(double measurement, double setpoint) {
    double rotationSpeed = rotationPID.calculate(measurement, setpoint);
    leftDriveMotor.set(rotationSpeed);
    rightDriveMotor.set(-rotationSpeed);
  }

  public void feedWatchdog() {
    m_drive.feed();
  }

  public boolean isOnTarget(double range, double setpoint) {
    if (range < 1.05*setpoint && range > .95*setpoint) {
      return true;
    }
    else {
      return false;
    }
  }

  public boolean isOnAngle(double yaw, double setpoint) {
    if (yaw < 2+setpoint && yaw > setpoint-2) {
      return true;
    }
    else {
      return false;
    }
  }
}
