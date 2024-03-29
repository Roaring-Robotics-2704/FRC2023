// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomus;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class automove extends CommandBase {
  private final Drivetrain m_Drivetrain;
  private double m_speed;
  private double m_turn;
  private double m_strafe; 
  /** Creates a new automove. */
  public automove(double speed , double turn,double strafe,Drivetrain subystem) {
    m_Drivetrain=subystem;
    m_speed= speed;
    m_turn = turn;
    m_strafe = strafe; 
  
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(m_Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Drivetrain.driveCartesian(m_speed, m_strafe, m_turn, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.driveCartesian(0, 0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
