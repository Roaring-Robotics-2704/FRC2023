// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MecanumDrivetrain extends SubsystemBase {
  /** Creates a new MecanumDrivetrain. */
  public MecanumDrivetrain() {}

  private WPI_TalonSRX m_frontrightMotor = new WPI_TalonSRX(Constants.c_frontrightDriveMotor);
  private WPI_TalonSRX m_backrightMotor = new WPI_TalonSRX(Constants.c_backrightDriveMotor);
  
  private WPI_VictorSPX m_frontleftMotor = new WPI_VictorSPX(Constants.c_frontleftDriveMotor);
  private WPI_VictorSPX m_backleftMotor = new WPI_VictorSPX(Constants.c_backleftDriveMotor);

  private MotorControllerGroup leftMotors = new MotorControllerGroup(m_backleftMotor, m_frontleftMotor);
  private MotorControllerGroup rightMotors = new MotorControllerGroup(m_backrightMotor, m_frontrightMotor);

  private MecanumDrive drive = new MecanumDrive(m_frontleftMotor, m_backleftMotor, m_frontrightMotor, m_backrightMotor);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void driveCartesian(double y, double x, double z){
    //negative stuff for demobot, all positive on comp bot
    drive.driveCartesian(-y,x,-z);
  }

  
}
