// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyroscope;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class GyroDrive extends CommandBase {
  /** Creates a new DriveRobot. */
  private Drivetrain driveTrain;
  private Gyroscope gyroscope;
  
  public GyroDrive(Drivetrain driveTrain, Gyroscope gyroscope) {
    
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
    
    //balancing variables
    //If gyro moved on robot
    //boolean autoBalanceXMode = false; //same as X
    boolean autoBalanceYMode = false; //same as Y

    final double MaximumAllowedAngle = 2.5; //is the maximum allowed in order to be consider engaged and docked(level)
    final double TargetAngle  = 2; //this number might need to be changed, it is when we stop adjusting

    //If gyro moved on robot
    //double gyroscopeXAngleDegrees = Gyroscope.zeroGyroX(Robot.gyroStartAngle);
    //SmartDashboard.putNumber("newStartAngle inital X", gyroscopeXAngleDegrees);
    double gyroscopeYAngleDegrees = Gyroscope.zeroGyroY(Robot.gyroStartAngle); //is after zeroing
    SmartDashboard.putNumber("newStartAngle inital Y", gyroscopeYAngleDegrees);
 
    double yAxisRate = 0;
    double zAxisRate = 0;

    //stablizing variables
    final double allowedTurnAngle = 2;
    double stabilizingSpeed = 0.1;

    PIDController stablizePID = new PIDController(Constants.GyroConstants.c_balanceKp, Constants.GyroConstants.c_balanceKi, Constants.GyroConstants.c_balanceKd);
    double correction = stablizePID.calculate(Gyroscope.gyro.getAngle());
    SmartDashboard.putNumber("PID correction", correction);
    
      //autobalance code
      //If gyro moved on robot 
      /*if ( !autoBalancePitchMode && (Math.abs(gyroscopeXAngleDegrees) >= Math.abs(MaximumAllowedAngle))) {
        autoBalancePitchMode = true;
      }
      else if ( autoBalancePitchMode && (Math.abs(gyroscopeXAngleDegrees) <= Math.abs(TargetAngle))) {
        autoBalancePitchMode = false;
      }*/

      if ( !autoBalanceYMode && (Math.abs(gyroscopeYAngleDegrees) >= Math.abs(MaximumAllowedAngle))) {
        autoBalanceYMode = true;
      }
      else if ( autoBalanceYMode && (Math.abs(gyroscopeYAngleDegrees) <= Math.abs(TargetAngle))) {
        autoBalanceYMode = false;
      }

      /*if ( autoBalancePitchMode ) {
        //If gyro moved on robot
        double minPower = 0.18;
        double maxPower = 0.25;
        double powerRange = maxPower - minPower;

        double minAngle = 2.5;
        double maxAngle = 10;
        double angleRange = maxAngle - minAngle;

        double multiplier = angleRange/100;
        double absAngle = Math.abs(gyroscopeXAngleDegrees);
        SmartDashboard.putNumber("absAngle", absAngle);
        double scaledPower = (absAngle - minPower)*(multiplier)*(powerRange);
        SmartDashboard.putNumber("scaledPower", scaledPower);
        double finalPower = minPower + scaledPower;
        SmartDashboard.putNumber("finalPower", finalPower);

        if(gyroscopeXAngleDegrees > 0){
          if(gyroscopeXAngleDegrees > maxAngle){
            yAxisRate = maxPower;
          }
          else if(gyroscopeXAngleDegrees < minAngle){
            yAxisRate = minPower;
          }
          else{
            yAxisRate = finalPower;
            SmartDashboard.putNumber("+xAxisRate", finalPower);
          }
        }
        else if(gyroscopeXAngleDegrees < 0){
          if(gyroscopeXAngleDegrees < -maxAngle){
            yAxisRate = -maxPower;
          }
          else if(gyroscopeXAngleDegrees > -minAngle){
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
      }*/
       
      if ( autoBalanceYMode ) {
        double minPower = 0.18;
        double maxPower = 0.38;
        double powerRange = maxPower - minPower;

        double minAngle = 2.5;
        double maxAngle = 11;
        double angleRange = maxAngle - minAngle;

        double multiplier = angleRange/100;
        double absAngle = Math.abs(gyroscopeYAngleDegrees);
        SmartDashboard.putNumber("absAngle", absAngle);
        double scaledPower = (absAngle - minPower)*(multiplier)*(powerRange);
        SmartDashboard.putNumber("scaledPower", scaledPower);
        double finalPower = minPower + scaledPower;
        SmartDashboard.putNumber("finalPower", finalPower);

        if(gyroscopeYAngleDegrees > 0){
          if(gyroscopeYAngleDegrees > maxAngle){
            yAxisRate = maxPower;
          }
          else if(gyroscopeYAngleDegrees < minAngle){
            yAxisRate = minPower;
          }
          else{
            yAxisRate = finalPower;
            SmartDashboard.putNumber("+xAxisRate", finalPower);
          }
        }
        else if(gyroscopeYAngleDegrees < 0){
          if(gyroscopeYAngleDegrees < -maxAngle){
            yAxisRate = -maxPower;
          }
          else if(gyroscopeYAngleDegrees > -minAngle){
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
      }

      //stablizing code
      /* 
        if(Gyroscope.gyro.getAngle() > allowedTurnAngle){
          zAxisRate = stabilizingSpeed;
          SmartDashboard.putNumber("stabilizingSpeed", zAxisRate);
        }
        else{
          zAxisRate = 0;
        }
        if(Gyroscope.gyro.getAngle() < -allowedTurnAngle){
          zAxisRate = -stabilizingSpeed;
          SmartDashboard.putNumber("-stabilizingSpeed", zAxisRate);
        }
        else{
          zAxisRate = 0;
        }
      */

      //PID stablizing
        if(Gyroscope.gyro.getAngle() > allowedTurnAngle){
          zAxisRate = -correction;
          SmartDashboard.putNumber("stabilizingSpeed", zAxisRate);
        }
        else{
          zAxisRate = 0;
        }
      
        if(Gyroscope.gyro.getAngle() < -allowedTurnAngle){
          zAxisRate = correction;
          SmartDashboard.putNumber("-stabilizingSpeed", zAxisRate);
        }
        else{
          zAxisRate = 0;
        }

      RobotContainer.m_Drivetrain.driveCartesian(yAxisRate, 0,zAxisRate);
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
