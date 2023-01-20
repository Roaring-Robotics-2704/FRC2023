// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  //Drive Train Motors
    //Declare Motor Controllers 
    //TalonSRXs
    private WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(Constants.DriveTrain.c_frontLeftMotor);
    private WPI_TalonSRX m_backLeft = new WPI_TalonSRX(Constants.DriveTrain.c_backLeftMotor);
    private WPI_TalonSRX m_frontRight = new WPI_TalonSRX(Constants.DriveTrain.c_frontRightMotor);
    private WPI_TalonSRX m_backRight = new WPI_TalonSRX(Constants.DriveTrain.c_backRightMotor);

    //Mecanum Drive Consturctor 
    private MecanumDrive drive = new MecanumDrive(m_frontLeft, m_backLeft, m_frontRight, m_backRight);
 
  public Drivetrain() {}

  public void driveCartesian(double yAxisSpeed, double xAxisSpeed, double zAxisSpeed){
    drive.driveCartesian(yAxisSpeed, -xAxisSpeed, zAxisSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
