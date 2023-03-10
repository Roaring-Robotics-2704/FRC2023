// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Mechanum.mecanum;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  //Drive Train Motors

  private WPI_TalonSRX m_backrightMotor = new WPI_TalonSRX(Constants.c_backrightDriveMotor);
  private WPI_TalonSRX m_frontrightMotor = new WPI_TalonSRX(Constants.c_frontrightDriveMotor);
  private WPI_TalonSRX m_frontleftMotor = new WPI_TalonSRX(Constants.c_frontleftDriveMotor);
  private WPI_TalonSRX m_backleftMotor = new WPI_TalonSRX(Constants.c_backleftDriveMotor);

ErrorCode br = m_backrightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
ErrorCode bl = m_backleftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
ErrorCode fr = m_frontrightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
ErrorCode fl = m_frontleftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

public void updatevalues() {
  double[] Array = new double[]{m_frontleftMotor.getSelectedSensorVelocity(), m_frontrightMotor.getSelectedSensorVelocity(),m_backleftMotor.getSelectedSensorVelocity(), m_backrightMotor.getSelectedSensorVelocity()}; 
  SmartDashboard.putNumberArray("wheelspeeds", Array);
}

  private MecanumDrive mecanumdrive = new MecanumDrive(m_frontleftMotor, m_backleftMotor, m_frontrightMotor, m_backrightMotor);
  public void driveCartesian(double y, double x, double z,double rotation){
    MecanumDriveWheelSpeeds speeds = mecanum.calculateSpeeds(mecanum.createSpeeds(y,x,z));
    mecanum.veloDrive(
    speeds,
    m_frontleftMotor,
    m_frontrightMotor,
    m_backleftMotor,
    m_backrightMotor,
    mecanumdrive
    );
    /*Rotation2d heading = Rotation2d.fromDegrees(rotation);
    mecanumdrive.driveCartesian(y,x,z,heading);*/
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
