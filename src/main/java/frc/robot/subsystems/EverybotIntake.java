// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EverybotIntake extends SubsystemBase {
  /** Creates a new EverybotIntake. */
  //Create new CANSparkMax
  CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeConstants.c_intakeMotor, MotorType.kBrushless);
  
  public EverybotIntake() {}

  public void setIntakeMotor(double percent, int amps) {
    intakeMotor.set(percent);
    intakeMotor.setSmartCurrentLimit(amps);
    SmartDashboard.putNumber("intake power (%)", percent);
    SmartDashboard.putNumber("intake motor current (amps)", intakeMotor.getOutputCurrent());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
