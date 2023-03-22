// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.pathplanner.lib.PathPlannerTrajectory;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

@AutoLog
public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  //Drive Train Motors
// Locations of the wheels relative to the robot center.


  private WPI_TalonSRX m_backrightMotor = new WPI_TalonSRX(Constants.c_backrightDriveMotor);
  private WPI_TalonSRX m_frontrightMotor = new WPI_TalonSRX(Constants.c_frontrightDriveMotor);
  private WPI_TalonSRX m_frontleftMotor = new WPI_TalonSRX(Constants.c_frontleftDriveMotor);
  private WPI_TalonSRX m_backleftMotor = new WPI_TalonSRX(Constants.c_backleftDriveMotor);
  
  ErrorCode frontRightEncoder = m_frontrightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
 
 ErrorCode frontLeftEncoder = m_frontleftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
   double frontLeftEncoderDistance = (m_frontleftMotor.getSelectedSensorPosition()/360)*0.59817;
   double frontLeftEncoderRate = m_frontleftMotor.getSelectedSensorVelocity();
 ErrorCode backRightEncoder = m_backrightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
   double backRightEncoderDistance = (m_backrightMotor.getSelectedSensorPosition()/360)*0.59817;
   double backRightEncoderRate = m_backleftMotor.getSelectedSensorVelocity();
 ErrorCode backLeftEncoder = m_backleftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
   double backLeftEncoderDistance = (m_backleftMotor.getSelectedSensorPosition()/360)*0.59817;
   double backLeftEncoderRate = m_backleftMotor.getSelectedSensorVelocity();
double frontRightEncoderDistance = (m_frontrightMotor.getSelectedSensorPosition()/360)*0.59817;//basically a verison of the get.distance()from the wpilib encoder class.
   Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
   Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
   Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
   Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
   
   // Creating my kinematics object using the wheel locations.
   MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
     m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
   );
   
   // Creating my odometry object from the kinematics object and the initial wheel positions.
   // Here, our starting pose is 5 meters along the long end of the field and in the
   // center of the field along the short end, facing the opposing alliance wall.
   MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(
     m_kinematics,
     Rotation2d.fromDegrees(-RobotContainer.m_imu.getAngle()),
     new MecanumDriveWheelPositions(
      (m_frontleftMotor.getSelectedSensorPosition()/360)*0.59817,(m_frontrightMotor.getSelectedSensorPosition()/360)*0.59817,
      (m_backleftMotor.getSelectedSensorPosition()/360)*0.59817, (m_backrightMotor.getSelectedSensorPosition()/360)*0.59817
     )
   );
   
  private MecanumDrive mecanumdrive = new MecanumDrive(m_frontleftMotor, m_backleftMotor, m_frontrightMotor, m_backrightMotor);
  public void driveCartesian(double y, double x, double z,double rotation){
    Rotation2d heading = Rotation2d.fromDegrees(rotation);
    mecanumdrive.driveCartesian(y,x,z,heading);
    

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var wheelPositions = new MecanumDriveWheelPositions(
      (m_frontleftMotor.getSelectedSensorPosition()/360)*0.59817,(m_frontrightMotor.getSelectedSensorPosition()/360)*0.59817,
      (m_backleftMotor.getSelectedSensorPosition()/360)*0.59817, (m_backrightMotor.getSelectedSensorPosition()/360)*0.59817
     );
  
    // Get the rotation of the robot from the gyro.
    var gyroAngle = Rotation2d.fromDegrees(-RobotContainer.m_imu.getAngle());
  
    // Update the pose
    Pose2d m_pose = m_odometry.update(gyroAngle, wheelPositions);
    Logger log = Logger.getInstance();
    log.recordOutput("pose", m_pose);
    log.recordOutput("motorSpeeds/frontleft", m_frontleftMotor.getSelectedSensorVelocity());
    log.recordOutput("motorSpeeds/frontright", m_frontrightMotor.getSelectedSensorVelocity());
    log.recordOutput("motorSpeeds/backleft", m_backleftMotor.getSelectedSensorVelocity());
    log.recordOutput("motorSpeeds/backright", m_backrightMotor.getSelectedSensorVelocity());
    log.recordOutput("UI/xbox/rotation", RobotContainer.xbox.getLeftX());
    log.recordOutput("UI/xbox/Y", RobotContainer.xbox.getRightY());
    log.recordOutput("UI/xbox/X", RobotContainer.xbox.getRightX());
    log.recordOutput("gyroAngle", -RobotContainer.m_imu.getAngle());
  }
 }
  

