// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.NormalDrivetrain;

public class DriveWithGyro extends CommandBase {
  /** Creates a new DriveWithGyro. */
  private NormalDrivetrain driveTrain;

  public DriveWithGyro(NormalDrivetrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.driveTrain =  driveTrain;
    addRequirements(driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //is working for demobot
    double xAxisRate = -RobotContainer.m_driverJoystick.getX(); 
    double yAxisRate = -RobotContainer.m_driverJoystick.getY();
    double zAxisRate = RobotContainer.m_driverJoystick.getZ();

    RobotContainer.m_driveTrain.driveCartesian(yAxisRate, xAxisRate, zAxisRate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
