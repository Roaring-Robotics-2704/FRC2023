// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.util.Units;

public class Vision extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("OV5647");

  PhotonPipelineResult result = camera.getLatestResult();

  /** Creates a new Vision. */
  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getTargetDistance() {
      
    double range =
                         PhotonUtils.calculateDistanceToTargetMeters(
                                 Constants.cameraHeightMeters,
                                 Constants.targetHeightMeters,
                                 Constants.cameraPitchRadians,
                                 Units.degreesToRadians(result.getBestTarget().getPitch()));
     return range; 
  }

  public double getTargetAngle() {
      double targetYaw = result.getBestTarget().getYaw();
      return targetYaw;
  }

  public boolean checkForTargets() {
    if (result.hasTargets()) {
      return true;
    }
    else {
      return false;
    }
  }
}
