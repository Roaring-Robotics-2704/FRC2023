// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Gyroscope;
import frc.robot.subsystems.NormalDrivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class AutoBalanceWithGyro extends CommandBase {
  /** Creates a new DriveRobot. */
  private NormalDrivetrain driveTrain;
  private Gyroscope gyroscope;
  
  public AutoBalanceWithGyro(NormalDrivetrain driveTrain, Gyroscope gyroscope) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain =  driveTrain;
    this.gyroscope = gyroscope;
    addRequirements(driveTrain);
    addRequirements(gyroscope);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Gyroscope.reset();
  }

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
    //works with the scaling code not the previous code, might need to add auto zeroing at start

   //gyroscope.gyro.reset();
    //gyroscope.gyro.calibrate();
    
    boolean autoBalancePitchMode = false; //same as X
    boolean autoBalanceRollMode = false; //same as Y

    final double MaximumAllowedAngle = 2.5; //is the maximum allowed in order to be consider engaged and docked(level)
    final double TargetAngle  = 2; //this number might need to be changed, it is when we stop adjusting

    //double pitchAngleDegrees = Gyroscope.zeroGyroX(Robot.gyroStartAngle);
    //SmartDashboard.putNumber("newStartAngle inital X", pitchAngleDegrees);
    double rollAngleDegrees = Gyroscope.zeroGyroY(Robot.gyroStartAngle);
    SmartDashboard.putNumber("newStartAngle inital Y", rollAngleDegrees);
 
    //double xAxisRate = RobotContainer.m_driverJoystick.getX(); //not nessary for just autoleveling
    //double yAxisRate = RobotContainer.m_driverJoystick.getY(); //not nessary for just autoleveling
    //double xAxisRate = 0;
    double yAxisRate = 0;
    
      /*if ( !autoBalancePitchMode && (Math.abs(pitchAngleDegrees) >= Math.abs(MaximumAllowedAngle))) {
        autoBalancePitchMode = true;
      }
      else if ( autoBalancePitchMode && (Math.abs(pitchAngleDegrees) <= Math.abs(TargetAngle))) {
        autoBalancePitchMode = false;
      }*/

      if ( !autoBalanceRollMode && (Math.abs(rollAngleDegrees) >= Math.abs(MaximumAllowedAngle))) {
        autoBalanceRollMode = true;
      }
      else if ( autoBalanceRollMode && (Math.abs(rollAngleDegrees) <= Math.abs(TargetAngle))) {
        autoBalanceRollMode = false;
      }

      if ( autoBalancePitchMode ) {
        //code if using the x axis
        /*double minPower = 0.18;
        double maxPower = 0.25;
        double powerRange = maxPower - minPower;

        double minAngle = 2.5;
        double maxAngle = 10;
        double angleRange = maxAngle - minAngle;

        double multiplier = angleRange/100;
        double absAngle = Math.abs(pitchAngleDegrees);
        SmartDashboard.putNumber("absAngle", absAngle);
        double scaledPower = (absAngle - minPower)*(multiplier)*(powerRange);
        SmartDashboard.putNumber("scaledPower", scaledPower);
        double finalPower = minPower + scaledPower;
        SmartDashboard.putNumber("finalPower", finalPower);

        if(pitchAngleDegrees > 0){
          if(pitchAngleDegrees > maxAngle){
            xAxisRate = maxPower;
          }
          else if(pitchAngleDegrees < minAngle){
            xAxisRate = minPower;
          }
          else{
            xAxisRate = finalPower;
            SmartDashboard.putNumber("+xAxisRate", finalPower);
          }
        }
        else if(pitchAngleDegrees < 0){
          if(pitchAngleDegrees < -maxAngle){
            xAxisRate = -maxPower;
          }
          else if(pitchAngleDegrees > -minAngle){
            xAxisRate = -minPower;
          }
          else{
            xAxisRate = -finalPower;
            SmartDashboard.putNumber("-xAxisRate", finalPower);
          }
        }
        else{
          yAxisRate = 0;
        }*/

        //if pitch is forward and back, worked with canibalized frc2023
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

        //old radian code that will not use since swiched to scaling, was for the pitch angle
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
        //if roll if forward and back, working scaling code just needs to be tested on the ground, worked while on blocks
        //not had a chance to test on the ground since issue with gyro readings
        double minPower = 0.18;
        double maxPower = 0.29;
        double powerRange = maxPower - minPower;

        double minAngle = 2.5;
        double maxAngle = 7;
        double angleRange = maxAngle - minAngle;

        double multiplier = angleRange/100;
        double absAngle = Math.abs(rollAngleDegrees);
        SmartDashboard.putNumber("absAngle", absAngle);
        double scaledPower = (absAngle - minPower)*(multiplier)*(powerRange);
        SmartDashboard.putNumber("scaledPower", scaledPower);
        double finalPower = minPower + scaledPower;
        SmartDashboard.putNumber("finalPower", finalPower);

        if(rollAngleDegrees > 0){
          if(rollAngleDegrees > maxAngle){
            yAxisRate = maxPower;
          }
          else if(rollAngleDegrees < minAngle){
            yAxisRate = minPower;
          }
          else{
            yAxisRate = finalPower;
            SmartDashboard.putNumber("+xAxisRate", finalPower);
          }
        }
        else if(rollAngleDegrees < 0){
          if(rollAngleDegrees < -maxAngle){
            yAxisRate = -maxPower;
          }
          else if(rollAngleDegrees > -minAngle){
            yAxisRate = -minPower;
          }
          else{
            yAxisRate = -finalPower;
            SmartDashboard.putNumber("-xAxisRate", finalPower);
          }
        }
        else{
          yAxisRate = 0;
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
      //should add pid so slowly go up, done is in the scaling code 
      //in the first spot is always where the new speed goes
      RobotContainer.m_driveTrain.driveCartesian(yAxisRate, 0,0);
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
