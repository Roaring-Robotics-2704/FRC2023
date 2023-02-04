// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightStrand extends SubsystemBase {
  public AddressableLED m_led = new AddressableLED(Constants.c_ledPort);
  public AddressableLEDBuffer m_LedBuffer = new AddressableLEDBuffer(Constants.c_lightLength);

  /** Creates a new LightStrand. */
  public LightStrand() {


    // Reuse buffer

    // Default to a length of 60, start empty output

    // Length is expensive to set, so only set it once, then just update data

    m_led.setLength(m_LedBuffer.getLength());

    // Set the data

    m_led.setData(m_LedBuffer);

    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //the following four methods provide multiple ways of setting individual leds. 
  //RGB and HSV, with either an array or just separate integers

  public void setRGB(int pos,int[] rgb){
    m_LedBuffer.setRGB(pos, rgb[0],rgb[1],rgb[2]);
  }

  public void setRGB(int pos,int r,int g,int b){
    m_LedBuffer.setRGB(pos, r, g, b);
  }

  public void setHSV(int pos,int[] hsv){
    m_LedBuffer.setHSV(pos, hsv[0],hsv[1],hsv[2]);
  }

  public void setHSV(int pos,int h, int s, int v){
    m_LedBuffer.setHSV(pos, h, s, v);
  }

  //actually sets all the leds that have been changed
  public void update(){
    m_led.setData(m_LedBuffer);
  }
}
