package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Auto extends CommandBase{
   public Auto() {
    addRequirements(RobotContainer.m_Drivetrain);
   } 
   ADIS16470_IMU gyro = RobotContainer.m_imu;
   private void moveAuto(double y,double x,double z) {
    RobotContainer.m_Drivetrain.driveCartesian(y,x,z,-gyro.getAngle());   
    }
Timer autoTime = new Timer();
public int mode;
@Override
public void initialize(){
    autoTime.reset();
    gyro.calibrate();
    gyro.reset();
    
}
double outputz;
PIDController align = new PIDController(Constants.zpid.p,Constants.zpid.i,Constants.zpid.d);
@Override
public void execute() {
    autoTime.start();
    SmartDashboard.putNumber("autoTime",autoTime.get());
    if (mode == 1) {//square
        while (autoTime.get() <= 1){//backwards
            moveAuto(-0.3,0,0);
        }
        
        while ( autoTime.get() <= 2){//right
            moveAuto(0,0.3,0);
        }
        
        while  ( autoTime.get() <= 3){//forwards
            moveAuto(0.3,0,0);
        }
        while  ( autoTime.get() <= 4){//left
            moveAuto(0,-0.3,0);
        }
    }          
    else if (mode == 2) {//backwards
        while ( autoTime.get() <= 1.65){
            moveAuto(01,0,0);
            System.out.println(RobotContainer.m_Drivetrain.getAbsoluteAngle());
            
        
        }
    }
    else if (mode == 3) {//back,sideways,back
        while (autoTime.get()<= 5.5){
            moveAuto(0.5,0,0);
        }
        while (autoTime.get()<=10){
            moveAuto(0,-0.5,0);
        }
        while(autoTime.get()<= 15){
            moveAuto(-0.5,0,0);
        }
    
    }
    else if (mode == 4) {//spinning square
        while (autoTime.get()<=2) {//back
           moveAuto(-0.3,0,0.3);
        }
        while (autoTime.get()<=4) {//right
            moveAuto(0,0.3,0.3);
        }
        while (autoTime.get()<=6) {//up
            moveAuto(0.3,0,0.3);
        }
        while (autoTime.get()<=8) {//left
            moveAuto(0,-0.3,0.3);
        }
    }
    else if (mode == 5) {
        while (autoTime.get() <= 1.7) {
            moveAuto(0,0,0.3);}
        while (autoTime.get()>=1.7) 
            while (gyro.getAngle() != 0) {
            moveAuto(0,0,align.calculate(gyro.getAngle(),0));
        }
    }
    else if (mode == 6) {//backup with encoder
        while (RobotContainer.m_Drivetrain.getAbsoluteAngle() <= 2235) {
            moveAuto(0.6, 0, 0);
        }
    }
}

@Override
public boolean isFinished(){
    return false;

    
}}

