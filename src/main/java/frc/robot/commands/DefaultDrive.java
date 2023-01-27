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

public class DefaultDrive extends CommandBase {
  /** Creates a new DefaultDrive. */
  private final Drivetrain m_drivetrain;
  private final Vision m_vision;
  private final XboxController m_xbox;
  double range;

  public DefaultDrive(Drivetrain drivetrain, Vision vision, XboxController xbox) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_vision = vision;
    m_xbox = xbox;

    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    PhotonPipelineResult result = m_vision.camera.getLatestResult();

    SmartDashboard.putBoolean("aaaaaa", result.hasTargets());

    if (result.hasTargets()) {
      range =
      PhotonUtils.calculateDistanceToTargetMeters(
              Constants.cameraHeightMeters,
              Constants.targetHeightMeters,
              Constants.cameraPitchRadians,
              Units.degreesToRadians(result.getBestTarget().getPitch()));
      SmartDashboard.putNumber("range", range);
      // SmartDashboard.updateValues();
    }

   if (m_xbox.getRawButton(4)) {
    if (result.hasTargets()) {
      m_drivetrain.distanceDrivingPID(range, 1.00);
      m_drivetrain.feedWatchdog();
      SmartDashboard.putString("what is it doing", "vision ing");
    }
    else {
      m_drivetrain.driveRobot(m_xbox.getLeftY(), m_xbox.getRightX());
      SmartDashboard.putString("what is it doing", "driving");
    }
   }
   else if (m_xbox.getRawButton(1)) {
    if (result.hasTargets()) {
      m_drivetrain.rotationDrivingPID(result.getBestTarget().getYaw(), 0);
      m_drivetrain.feedWatchdog();
    }
    else {
      m_drivetrain.driveRobot(m_xbox.getLeftY(), m_xbox.getRightX());
      m_drivetrain.feedWatchdog();
    }
   }
   else {
    m_drivetrain.driveRobot(m_xbox.getLeftY(), m_xbox.getRightX());
    SmartDashboard.putString("what is it doing", "driving");
   }
   
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
