// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import frc.robot.Constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.*;

import org.ejml.equation.IntegerSequence.Range;

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

    if (result.hasTargets()) {
      range =
      PhotonUtils.calculateDistanceToTargetMeters(
              Constants.cameraHeightMeters,
              Constants.targetHeightMeters,
              Constants.cameraPitchRadians,
              Units.degreesToRadians(m_vision.result.getBestTarget().getPitch()));
      SmartDashboard.putNumber("range", range);
      // SmartDashboard.updateValues();
    }
   if (m_xbox.getRawButton(4)) {
    if (m_vision.checkForTargets()) {
      m_drivetrain.distanceDrivingPID(range, 1.25);
      m_drivetrain.feedWatchdog();
    }
    else {
      m_drivetrain.driveRobot(m_xbox.getLeftY(), m_xbox.getRightX());
    }
   }
   else if (m_xbox.getRawButton(1)) {
    if (m_vision.checkForTargets()) {
      m_drivetrain.rotationDrivingPID(m_vision.getTargetAngle(), 0);
      m_drivetrain.feedWatchdog();
    }
    else {
      m_drivetrain.driveRobot(m_xbox.getLeftY(), m_xbox.getRightX());
    }
   }
   else {
    m_drivetrain.driveRobot(m_xbox.getLeftY(), m_xbox.getRightX());
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
