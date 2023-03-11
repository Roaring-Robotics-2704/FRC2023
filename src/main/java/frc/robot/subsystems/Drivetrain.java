// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Mechanum.mecanum;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  //Drive Train Motors


  public void driveCartesian(double y, double x, double z){
    mecanum.driveCartesian(y,x,z);
    //mecanum.veloDrive(y,x,z);
    /*Rotation2d heading = Rotation2d.fromDegrees(rotation);
    mecanumdrive.driveCartesian(y,x,z,heading);*/
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
