// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LightCommand extends CommandBase {
  /** Creates a new LightCommand. */
  public LightCommand() {
    addRequirements(RobotContainer.m_LightStrand);
  }

  Timer lightTimer = new Timer();

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lightTimer.reset();
    lightTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {//if chain should be kept in sync with lightChooser options
  }

  private void setFullStrand(int r, int g, int b) {
    for(int i = 0; i<Constants.c_lightLength; i++){
      RobotContainer.m_LightStrand.setRGB(i,r,g,b);
    }
  }

  void Dynamic_Alliance(){
    if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
      setFullStrand(255, 0, 0);
    }
    else if(DriverStation.getAlliance() == DriverStation.Alliance.Blue){
      setFullStrand(0, 0, 255);
    }
  }

  public void sweep(int position){ //not fully understood, result of trial and error
    int direction = 1;

    if((position/Constants.c_lightLength)%2==1){//if on even trip
      direction *= -1; //invert direction
    }

    position = position % Constants.c_lightLength; //so position is bounded

    if (direction == 1){ // sweep forward
      SmartDashboard.putString("sweepdir", "forward");
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
      SmartDashboard.putString("sweepdir", "back");
      for (var i = 0;i<Constants.c_lightLength;i++){
        if(i<Constants.c_lightLength - position){
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
    
          int hue = (firstPixelHue + (i * 180 / Constants.c_lightLength)) % 180;
    
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
