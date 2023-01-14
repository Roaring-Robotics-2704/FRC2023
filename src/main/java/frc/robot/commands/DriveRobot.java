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

      }
      if ( autoBalanceRollMode ) {
        SmartDashboard.putNumber("YAxisRate", RobotContainer.m_driverJoystick.getY());
        double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
        yAxisRate = Math.sin(rollAngleRadians) * -1.5;
      }
      //should add pid so slowly go up 

      RobotContainer.m_driveTrain.driveCartesian(xAxisRate, yAxisRate,0);
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
