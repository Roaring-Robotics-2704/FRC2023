// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class LightCommand extends CommandBase {
  /** Creates a new LightCommand. */
  public LightCommand() {
    addRequirements(RobotContainer.m_LightStrand);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //rainbow(0);
    sweep((int)(Timer.getFPGATimestamp()*Constants.c_lightLength));
    RobotContainer.m_LightStrand.update();
  }

  public void sweep(int position){
    int direction = 1;
    if((position/Constants.c_lightLength)%2 == 0){ //alternating direction
      direction = -1;
    }

    position = position % Constants.c_lightLength;

    if (direction == 1){ // sweep forward
      for (var i = 0;i<Constants.c_lightLength;i++){
        if(i>position){
          RobotContainer.m_LightStrand.setRGB(i, Constants.yellowRGB);
        }
        else {
          RobotContainer.m_LightStrand.setRGB(i,Constants.blueRGB);
        }
      }
    }
    else{ //sweep back
      for (var i = 0;i<Constants.c_lightLength;i++){
        if(i>position){
          RobotContainer.m_LightStrand.setRGB(i, Constants.blueRGB);
        }
        else {
          RobotContainer.m_LightStrand.setRGB(i,Constants.yellowRGB);
        }
      }
    }
  }

  public void rainbow(int firstPixelHue){
        // For every pixel

        for (var i = 0; i < Constants.c_lightLength; i++) {

          // Calculate the hue - hue is easier for rainbows because the color
    
          // shape is a circle so only one value needs to precess
    
          var hue = (firstPixelHue + (i * 180 / Constants.c_lightLength)) % 180;
    
          // Set the value
    
          RobotContainer.m_LightStrand.setHSV(i, hue, 255, 12);
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
