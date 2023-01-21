// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyroscope;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveRobot extends CommandBase {
  /** Creates a new DriveRobot. */
  private Drivetrain driveTrain;
  private Gyroscope gyroscope;
  
  public DriveRobot(Drivetrain driveTrain, Gyroscope gyroscope) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain =  driveTrain;
    this.gyroscope = gyroscope;
    addRequirements(driveTrain);
    addRequirements(gyroscope);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //code that uses subsystem more, does not have scaling code
    /* 
    double joystickXInput = Gyroscope.getXRateValue();
    double joystickYInput = -Gyroscope.getYRateValue();

    RobotContainer.m_driveTrain.driveCartesian(joystickYInput, joystickXInput, 0);
    */

    //Plain Drivetrain
    /* 
    //all of the postiive and negatives are from testing the mecanum drive
    //make sure the lights are correct by electrially inverting the motors when needed
    //get joystickvalues
    double joystickXInput = -RobotContainer.m_driverJoystick.getX();
    double joystickYInput = -RobotContainer.m_driverJoystick.getY();
    double joystickZInput = RobotContainer.m_driverJoystick.getZ();

    //speed control
    joystickXInput *= 0.5;
    joystickYInput *= 0.5;
    joystickZInput  *= 0.5;

    //deadzone
    double deadzone = 0.2;
    double turnDeadzone = 0.25;

    if (Math.abs(RobotContainer.m_driverJoystick.getX()) < deadzone) {
      joystickXInput = 0;
    } else if (Math.abs(RobotContainer.m_driverJoystick.getY()) < deadzone) {
      joystickYInput = 0;
    } else if (Math.abs(RobotContainer.m_driverJoystick.getZ()) < turnDeadzone) {
      joystickZInput = 0;
    }
    
    //call driveCartesion from Drive Train
    RobotContainer.m_driveTrain.driveCartesian(joystickYInput, joystickXInput, joystickZInput);
    */

    //Gyro code that works but the speed of adjustment is not right
     
    boolean autoBalancePitchMode = false; //same as X
    boolean autoBalanceRollMode = false; //same as Y

    final double MaximumAllowedAngle = 2.5; //is the maximum allowed in order to be consider engaged and docked(level)
    final double TargetAngle  = 2; //this number might need to be changed, it is when we stop adjusting

    double pitchAngleDegrees = RobotContainer.m_gyroscope.gyro.getXComplementaryAngle();
    double rollAngleDegrees = RobotContainer.m_gyroscope.gyro.getYComplementaryAngle();
 
    double xAxisRate = RobotContainer.m_driverJoystick.getX();
    double yAxisRate = RobotContainer.m_driverJoystick.getY();
   
      if ( !autoBalancePitchMode && (Math.abs(pitchAngleDegrees) >= Math.abs(MaximumAllowedAngle))) {
        autoBalancePitchMode = true;
      }
      else if ( autoBalancePitchMode && (Math.abs(pitchAngleDegrees) <= Math.abs(TargetAngle))) {
        autoBalancePitchMode = false;
      }
      if ( !autoBalanceRollMode && (Math.abs(rollAngleDegrees) >= Math.abs(MaximumAllowedAngle))) {
        autoBalanceRollMode = true;
      }
      else if ( autoBalanceRollMode && (Math.abs(rollAngleDegrees) <= Math.abs(TargetAngle))) {
        autoBalanceRollMode = false;
      }

      if ( autoBalancePitchMode ) {
        //if x is movable, worked with canibalized frc2023
        /* 
        double minMotorPower = 0.2;
        double maxMotorPower = 0.5;
        double powerRange = maxMotorPower - minMotorPower;

        double maxAngle = 15;
        double minAngle = 2;
        double angleRange = maxAngle - minAngle;

        double multiplier = maxAngle / 100;
        double angleToMotorSpeed = angleRange * multiplier;
        double scaleMotorSpeed = angleToMotorSpeed * powerRange + minMotorPower;

        if(pitchAngleDegrees > 0){
          if(Math.abs(pitchAngleDegrees) > maxAngle){
            xAxisRate = - maxMotorPower;
          }
          else{
            xAxisRate = - scaleMotorSpeed;
          }
          //must be negative
        }
        else if(pitchAngleDegrees < 0){
          if(Math.abs(pitchAngleDegrees) > maxAngle){
            xAxisRate = maxMotorPower;
          }
          else{
            xAxisRate = scaleMotorSpeed;
          }
          //must be positive
          */
        }

        //old radian code that will not use since swiched to scaling
        /* 
        double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
        xAxisRate = Math.sin(pitchAngleRadians) * -1.5;
        //look to set minnimum and max for speeds
        SmartDashboard.putNumber("beforeXAxisRate", xAxisRate);
        if(xAxisRate > 0.3){
          xAxisRate = 0.3;
        }
        else if(xAxisRate < 0.18){
          xAxisRate = 0.18;
        }
        SmartDashboard.putNumber("XAxisRate", xAxisRate);
        */
       
      if ( autoBalanceRollMode ) {
        double minPower = 0.2;
        double maxPower = 0.5;
        double powerRange = maxPower - minPower;

        double minAngle = 2.5;
        double maxAngle = 10;
        double angleRange = maxAngle - minAngle;

        double multiplier = angleRange/100;
        double absAngle = Math.abs(rollAngleDegrees);
        double scaledPower = (absAngle - minPower)*(multiplier)*(powerRange);
        double finalPower = minPower + scaledPower;

        if(rollAngleDegrees > 0){
          if(rollAngleDegrees > maxAngle){
            xAxisRate = maxPower;
          }
          else if(rollAngleDegrees < minAngle){
            xAxisRate = minPower;
          }
          else{
            xAxisRate = finalPower;
            SmartDashboard.putNumber("+xAxisRate", finalPower);
          }
        }
        else if(rollAngleDegrees < 0){
          if(rollAngleDegrees < -maxAngle){
            xAxisRate = -maxPower;
          }
          else if(rollAngleDegrees > -minAngle){
            xAxisRate = -minPower;
          }
          else{
            xAxisRate = -finalPower;
            SmartDashboard.putNumber("-xAxisRate", finalPower);
          }
        }
        else{
          xAxisRate = 0;
        }


        //Balance code that does not work
        /* 
        double minMotorPower = 0.1;
        double maxMotorPower = 0.3;
        double powerRange = maxMotorPower - minMotorPower;

        double maxAngle = 15;
        double minAngle = 2;
        double angleRange = maxAngle - minAngle;
        System.out.println(angleRange);

        double multiplier = maxAngle / 100;
        System.out.println(multiplier);
        double angleToMotorSpeed = angleRange * multiplier;
        System.out.println(angleToMotorSpeed);
        double scaleMotorSpeed = angleToMotorSpeed * powerRange + minMotorPower;
        System.out.println(scaleMotorSpeed);

        if(rollAngleDegrees > 0){
          if(rollAngleDegrees > maxAngle){
            yAxisRate = maxMotorPower;
          }
          else{
            yAxisRate = scaleMotorSpeed;
            SmartDashboard.putNumber("+xmotorSpeed", scaleMotorSpeed);
          }
        }
        else if(rollAngleDegrees < 0){
          if(rollAngleDegrees > maxAngle){
            yAxisRate = -maxMotorPower;
          }
          else{
            yAxisRate = -scaleMotorSpeed;
            SmartDashboard.putNumber("x-motorSpeed", scaleMotorSpeed);
          }*/
        
        /*SmartDashboard.putNumber("YAxisRate", RobotContainer.m_driverJoystick.getY());
        double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
        yAxisRate = Math.sin(rollAngleRadians) * -1.5;*/
      }
      //should add pid so slowly go up 

      RobotContainer.m_driveTrain.driveCartesian(xAxisRate, 0,0);
      //RobotContainer.m_driveTrain.driveCartesian(xAxisRate, yAxisRate,0);
      

    //Gyro code that does not work, tried to use the subsystems
    /*double xAxisRate = -RobotContainer.m_driverJoystick.getX();
    double yAxisRate = -RobotContainer.m_driverJoystick.getY();

    if (RobotContainer.m_gyroscope.Pitch()) {
      System.out.println("pitch");
      double pitchAngleRadians = Gyroscope.pitchAngleDegrees * (Math.PI / 180.0);
      xAxisRate = Math.sin(pitchAngleRadians) * -1;
    }
    if (RobotContainer.m_gyroscope.getIfRollModeOn()) {
      double rollAngleRadians = Gyroscope.rollAngleDegrees * (Math.PI / 180.0);
      yAxisRate = Math.sin(rollAngleRadians) * -1;
    }
    RobotContainer.m_driveTrain.driveCartesian(yAxisRate, xAxisRate,0);
    */
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
