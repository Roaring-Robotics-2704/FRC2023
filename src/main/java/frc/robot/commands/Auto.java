package frc.robot.commands;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;


public class Auto extends CommandBase{
   public Auto() {
    addRequirements(RobotContainer.m_Drivetrain);
   } 
  
private void moveAuto(double y,double x,double z) {
RobotContainer.m_Drivetrain.driveCartesian(y,x,z,-RobotContainer.m_imu.getAngle()); 
}
Timer autoTime = new Timer();
public int mode;

@Override
public void initialize(){
    autoTime.reset();
    
}
double outputz;
@Override
public void execute() {
    autoTime.start();
    SmartDashboard.putNumber("autoTime",autoTime.get());
    if (mode == 1) {//square
        while (autoTime.get() <= 2.06){//backwards
            moveAuto(-0.6,0,0);
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
        while ( autoTime.get() <= 2.57){
            moveAuto(0.8,0,0);
        }
        
    }
    else if (mode == 3) {//back,sideways,back
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
        while (autoTime.get() <=1){
            moveAuto(0.3, 0, 0);
        }
        while (autoTime.get() <= 2){
            moveAuto(0,0.3,0);
        }
        while(autoTime.get() <=3 ){
            moveAuto(0.3, 0,  0 );
        }
    }
    else if (mode == 6){// on and off 
        while(autoTime.get()<= 3){// fowards over the chrage station 
            moveAuto(0.8, 0, 0);// speed needs to be at least 0.8 for the robot to climb the charge station due to the arm 
        }
        while (autoTime.get()<= 4.55){ // backwards onto the charge station until center 
            moveAuto(-0.8, 0, 0);
        }
    }
    else if (mode == 7){// right side balance 
        while(autoTime.get()<=3){// forwards 
            moveAuto(0.8,0,0);
        }
        while (autoTime.get()<=4.5){// left 
            moveAuto(0, -0.8, 0);
        }
        while (autoTime.get()<=5){// backwards 
            moveAuto(-0.8, 0,0 );
        }
        }
    else if (mode == 8){// left side balance 
        while(autoTime.get()<=3){// forwards 
                moveAuto(0.8,0,0);
            }
        while (autoTime.get()<=4.5){// right  
                moveAuto(0, 0.8, 0);
            }
        while (autoTime.get()<=4){// backwards 
                moveAuto(-0.8, 0,0 );
         }
         }
    else if (mode == 9){
        while(autoTime.get()<= 3){
            moveAuto(0, 0.8, 0);
        }
        while(autoTime.get()<= 3.5){
            moveAuto(0, 0, 0);
        }
    }
    }
    


@Override
public boolean isFinished(){
    return false;

    
}
}


