// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  //Drive Train Motors
  private WPI_TalonSRX m_backrightMotor = new WPI_TalonSRX(Constants.c_backrightDriveMotor);
  private WPI_TalonSRX m_frontrightMotor = new WPI_TalonSRX(Constants.c_frontrightDriveMotor);
  private WPI_TalonSRX m_frontleftMotor = new WPI_TalonSRX(Constants.c_frontleftDriveMotor);
  private WPI_TalonSRX m_backleftMotor = new WPI_TalonSRX(Constants.c_backleftDriveMotor);
  private MecanumDrive mecanumdrive = new MecanumDrive(m_frontleftMotor, m_backleftMotor, m_frontrightMotor, m_backrightMotor);
  public void driveCartesian(double y, double x, double z,double rotation){
    Rotation2d heading = Rotation2d.fromDegrees(rotation);
    mecanumdrive.driveCartesian(y,x,z,heading);
    
    /*ErrorCode frontRightEncoder = m_frontrightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    double frontRightEncoderDistance = m_frontrightMotor.getSelectedSensorPosition();//basically a verison of the get.distance()from the wpilib encoder class.
    double frontRightEncoderRate = m_frontrightMotor.getSelectedSensorVelocity();//basically a verison of the get.Rate()from the wpilib encoder class.
   ErrorCode frontLeftEncoder = m_frontleftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
     double frontLeftEncoderDistance = m_frontleftMotor.getSelectedSensorPosition();
     double frontLeftEncoderRate = m_frontleftMotor.getSelectedSensorVelocity();
   ErrorCode backRightEncoder = m_backrightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
     double backRightEncoderDistance = m_backrightMotor.getSelectedSensorPosition();
     double backRightEncoderRate = m_backleftMotor.getSelectedSensorVelocity();
   ErrorCode backLeftEncoder = m_backleftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
     double backLeftEncoderDistance = m_backleftMotor.getSelectedSensorPosition();
     double backLeftEncoderRate = m_backleftMotor.getSelectedSensorVelocity();*/

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
