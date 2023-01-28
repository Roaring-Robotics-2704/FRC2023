// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import frc.robot.Constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.*;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class OnlyDriving extends CommandBase {
  /** Creates a new OnlyDriving. */
  private final Drivetrain m_drivetrain;
  private final XboxController m_xbox;

  public OnlyDriving(Drivetrain drivetrain, XboxController xbox) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_xbox = xbox;

    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.driveRobot(m_xbox.getLeftY(), m_xbox.getRightX());
    SmartDashboard.putString("what is it doing", "driving");
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
