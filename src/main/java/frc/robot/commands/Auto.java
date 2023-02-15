package frc.robot.commands;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Gyroscope;

public class Auto extends CommandBase{
   public Auto() {
    addRequirements(RobotContainer.m_Drivetrain);
   } 
   ADIS16470_IMU gyro = Gyroscope.gyro;
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
        while ( autoTime.get() <= 4){
            moveAuto(-0.3,0,0);
        }
        while (autoTime.get()<=5){
            Gyroscope.autoBalanceRollModeOnOff();
           
        }
            
        
    }
    if (mode == 3) {//back,sideways,back
        while (autoTime.get()<= 2){
            moveAuto(-0.3,0,0);
        }
        while (autoTime.get()<=3){
            moveAuto(0,-0.3,0);
        }
        while(autoTime.get()<= 4){
            moveAuto(-0.3,0,0);
        }
    
    }
     else if (mode == 4) {//spinning square
        while (autoTime.get()<=1) {//back
           moveAuto(-0.3,0,0.3);
        }
        while (autoTime.get()<=2) {//right
            moveAuto(0,0.3,0.3);
        }
        while (autoTime.get()<=3) {//up
            moveAuto(0.3,0,0.3);
        }
        while (autoTime.get()<=4) {//left
            moveAuto(0,-0.3,0.3);
        }
    }
    else if (mode==5){
        while (autoTime.get()<=1){
            moveAuto(-0.3, 0, 0);
        }
        while (autoTime.get()<= 2){
            moveAuto(0,0.3,0);
        }
        while(autoTime.get()<= 3){

            Gyroscope.autoBalancePitchModeOnOff();
        }
    }
    
}

@Override
public boolean isFinished(){
    return false;

    
}
}