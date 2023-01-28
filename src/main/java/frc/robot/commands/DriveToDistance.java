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

public class DriveToDistance extends CommandBase {
  /** Creates a new DriveToDistance. */
  private final Drivetrain m_drivetrain;
  private final Vision m_vision;
  private final XboxController m_xbox;
  double range;

  public DriveToDistance(Drivetrain drivetrain, Vision vision, XboxController xbox) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_vision = vision;
    m_xbox = xbox;

    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
              Units.degreesToRadians(result.getBestTarget().getPitch()));
      SmartDashboard.putNumber("range", range);
    }
    if (result.hasTargets()) {
      m_drivetrain.distanceDrivingPID(range, Constants.distanceSetpoint);
      SmartDashboard.putString("what is it doing", "vision ing");
    }
    m_drivetrain.feedWatchdog();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_drivetrain.isOnTarget(range, Constants.distanceSetpoint)) {
      return true;
    }
    else {
      return false;
    }
  }
}
