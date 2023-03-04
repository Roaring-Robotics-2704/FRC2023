// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EverybotArm extends SubsystemBase {
  /** Creates a new EverybotArm. */
  //Create new TalonSRX
  public TalonSRX armMotor = new TalonSRX(Constants.ArmConstants.c_armMotor); 

  //Create new Encoder
  /*ErrorCode frontRightEncoder = armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  double frontRightEncoderDistance = armMotor.getSelectedSensorPosition();//like get.distance()
  double frontRightEncoderRate = armMotor.getSelectedSensorVelocity();//get.rate()
  double firstLevel = 3;

  //Create new PID Controller
  PIDController armPID = new PIDController(Constants.ArmConstants.c_armEncoderKp, Constants.ArmConstants.c_armEncoderKi, Constants.ArmConstants.c_armEncoderKd);
*/
  //public static final int ArmCurrentLimit = Constants.ArmConstants.c_armCurrentLimit; //Amps motor can use
  //public static final double ArmPower = Constants.ArmConstants.c_armPower; //Precent output when go up and down

  public EverybotArm() {}

  public void setArmMotor(double percent) {
    armMotor.set(TalonSRXControlMode.PercentOutput, percent);
    SmartDashboard.putNumber("arm power", percent);
  }

  /*public void setArmMotor(){
    double speed = armPID.calculate(frontRightEncoderDistance, firstLevel);
    armMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }*/
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
