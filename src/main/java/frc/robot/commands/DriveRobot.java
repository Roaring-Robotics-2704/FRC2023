// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveRobot extends CommandBase {
  /** Creates a new DriveRobot. */
  private Drivetrain driveTrain;
  
  public DriveRobot(Drivetrain driveTrain) {
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
    //all of the postiive and negatives are from testing the mecanum drive
    //make sure the lights are correct by electrially inverting the motors when needed
    //get joystickvalues
    double joystickXInput = RobotContainer.m_driverJoystick.getX();
    double joystickYInput = RobotContainer.m_driverJoystick.getY();
    double joystickZInput = RobotContainer.m_driverJoystick.getZ();

    //deadzone
    double deadzone = 0.25;
    double turnDeadzone = 0.25;

    if (Math.abs(RobotContainer.m_driverJoystick.getX()) < deadzone) {
      joystickXInput = 0;
    } else if (Math.abs(RobotContainer.m_driverJoystick.getY()) < deadzone) {
      joystickYInput = 0;
    } else if (Math.abs(RobotContainer.m_driverJoystick.getZ()) < turnDeadzone) {
      joystickZInput = 0;
    }
    
    //call driveCartesion from Drive Train
    RobotContainer.m_driveTrain.driveCartesian(joystickYInput, joystickXInput, joystickZInput);
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
