// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  //Drive Train Motors
  private WPI_TalonSRX m_frontrightMotor = new WPI_TalonSRX(Constants.c_frontrightDriveMotor);
  private WPI_TalonSRX m_backrightMotor = new WPI_TalonSRX(Constants.c_backrightDriveMotor);
  private WPI_TalonSRX m_frontleftMotor = new WPI_TalonSRX(Constants.c_frontleftDriveMotor);
  private WPI_TalonSRX m_backleftMotor = new WPI_TalonSRX(Constants.c_backleftDriveMotor);
  MotorControllerGroup m_right = new MotorControllerGroup(m_frontrightMotor, m_backrightMotor);
  MotorControllerGroup m_left = new MotorControllerGroup(m_frontleftMotor, m_backleftMotor);
  private MecanumDrive mecanumdrive = new MecanumDrive(m_frontleftMotor, m_backleftMotor, m_frontrightMotor, m_backrightMotor);
  private DifferentialDrive tankdrive = new DifferentialDrive(m_left, m_right);
  public void driveCartesian(double y, double x, double z,double rotation){
    Rotation2d heading = Rotation2d.fromDegrees(rotation);
    mecanumdrive.driveCartesian(y,x,z,heading);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
