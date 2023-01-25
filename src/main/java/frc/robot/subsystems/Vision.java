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

import edu.wpi.first.wpilibj.smartdashboard.*;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public Vision() {}
  
  public PhotonCamera camera = new PhotonCamera("OV5647");

  public PhotonPipelineResult result = camera.getLatestResult();

  public static double range;
  public int anne = 1;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("got targets??", result.hasTargets());
    
    // if (result.hasTargets()) {
    //   range =
    //   PhotonUtils.calculateDistanceToTargetMeters(
    //           Constants.cameraHeightMeters,
    //           Constants.targetHeightMeters,
    //           Constants.cameraPitchRadians,
    //           Units.degreesToRadians(result.getBestTarget().getPitch()));
    //   SmartDashboard.putNumber("range", range);
    //   // SmartDashboard.updateValues();
    // }
    // else {
    //   SmartDashboard.putNumber("range", 0);
    // }
    // anne ++;
    // SmartDashboard.putNumber("var", anne);
     
  }

  /*public double getTargetDistance() {
      
    double range =
                         PhotonUtils.calculateDistanceToTargetMeters(
                                 Constants.cameraHeightMeters,
                                 Constants.targetHeightMeters,
                                 Constants.cameraPitchRadians,
                                 Units.degreesToRadians(result.getBestTarget().getPitch()));
     return range; 
  }*/

  // public double getTargetDistance() {
  //   return range;
  // }

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
