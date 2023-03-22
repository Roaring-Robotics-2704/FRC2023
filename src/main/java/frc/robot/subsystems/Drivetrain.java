// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.ErrorCode;
//import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  //Drive Train Motors
  private WPI_TalonSRX m_backrightMotor = new WPI_TalonSRX(Constants.c_backrightDriveMotor);
  public WPI_TalonSRX m_frontrightMotor = new WPI_TalonSRX(Constants.c_frontrightDriveMotor);
  public WPI_TalonSRX m_frontleftMotor = new WPI_TalonSRX(Constants.c_frontleftDriveMotor);
  private WPI_TalonSRX m_backleftMotor = new WPI_TalonSRX(Constants.c_backleftDriveMotor);

  public ErrorCode frontRightEncoder = m_frontrightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
  ErrorCode frontLeftEncoder = m_frontleftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
  ErrorCode backrightEncoder = m_backrightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
  ErrorCode backleftEncoder = m_backleftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);

  PIDController distancePID = new PIDController(0.0001, 0, 0);

  private MecanumDrive mecanumdrive = new MecanumDrive(m_frontleftMotor, m_backleftMotor, m_frontrightMotor, m_backrightMotor);
  
  public void driveCartesian(double y, double x, double z,double rotation){
    Rotation2d heading = Rotation2d.fromDegrees(rotation);
    mecanumdrive.driveCartesian(y,x,z,heading);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveToDistance(double measurement, double setpoint) {
    double movementSpeed = distancePID.calculate(measurement, setpoint);
   // m_frontrightMotor.set(movementSpeed);
    m_frontleftMotor.set(movementSpeed);
   // m_backrightMotor.set(movementSpeed);
   // m_backleftMotor.set(movementSpeed);
  }

  public void zeroEncoders() {
    m_frontrightMotor.setSelectedSensorPosition(0);
    m_frontleftMotor.setSelectedSensorPosition(0);
    m_backleftMotor.setSelectedSensorPosition(0);
    m_backrightMotor.setSelectedSensorPosition(0);
  }

  public double readEncoder() {
    return m_frontleftMotor.getSelectedSensorPosition();
  }

}
