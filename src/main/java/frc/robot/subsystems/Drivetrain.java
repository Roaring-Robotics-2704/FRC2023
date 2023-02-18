// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  static Translation2d m_frontLeftLocation = new Translation2d(0.2286, 0.2286);
  static Translation2d m_frontRightLocation = new Translation2d(0.2286, -0.2286);
  static Translation2d m_backLeftLocation = new Translation2d(-0.2286, 0.2286);
  static Translation2d m_backRightLocation = new Translation2d(-0.2286, -0.2286);
  public static final MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics( m_frontLeftLocation,  m_frontRightLocation,  m_backLeftLocation,  m_backRightLocation);
  ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5);

  MecanumDriveWheelSpeeds wheelSpeeds = kDriveKinematics.toWheelSpeeds(speeds);
  double frontLeft = wheelSpeeds.frontLeftMetersPerSecond;
  double frontRight = wheelSpeeds.frontRightMetersPerSecond;
  double backLeft = wheelSpeeds.rearLeftMetersPerSecond;
  double backRight = wheelSpeeds.rearRightMetersPerSecond;
  //Drive Train Motors
  private WPI_TalonSRX m_backrightMotor = new WPI_TalonSRX(Constants.c_backrightDriveMotor);
  private WPI_TalonSRX m_frontrightMotor = new WPI_TalonSRX(Constants.c_frontrightDriveMotor);
  private WPI_TalonSRX m_frontleftMotor = new WPI_TalonSRX(Constants.c_frontleftDriveMotor);
  private WPI_TalonSRX m_backleftMotor = new WPI_TalonSRX(Constants.c_backleftDriveMotor);

  private MecanumDrive mecanumdrive = new MecanumDrive(m_frontleftMotor, m_backleftMotor, m_frontrightMotor, m_backrightMotor);
  public void driveCartesian(double y, double x, double z,double rotation){
    Rotation2d heading = Rotation2d.fromDegrees(rotation);
    mecanumdrive.driveCartesian(y,x,z,heading);
  }
  public void getencodervalue(int motorport) {
    WPI_TalonSRX _testMotor = new WPI_TalonSRX(motorport);
    _testMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    _testMotor.getSelectedSensorPosition();
    _testMotor.close();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
