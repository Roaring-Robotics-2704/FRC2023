package frc.robot.commands;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class Auto extends CommandBase{
   public Auto() {
    addRequirements(RobotContainer.m_Drivetrain);
   } 
  
private void moveAuto(double y,double x,double z) {
RobotContainer.m_Drivetrain.driveCartesian(y,x,z,0); 
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

    SmartDashboard.putNumber("autoTime",autoTime.get());
      autoTime.start();

    /*if (mode == 1) {//square
        if (autoTime.get() <= 3){//backwards
            moveAuto(-0.6,0,0);
        }
        
       else  if  ( autoTime.get() <= 6){//right
          moveAuto(0,0.3,0);
        }
        
        else if ( autoTime.get() <= 9){//forwards
            moveAuto(0.3,0,0);
        }
       else if ( autoTime.get() <= 12){//left
            moveAuto(0,-0.3,0);
        }
    }   */ 
     if (mode == 2) {//forwards
        if ( autoTime.get() <= 2.5025){
            moveAuto(-0.8,0,0);
        }
        
    }
    else if (mode == 3) {
        if (autoTime.get()<= 1){// fowards over the chrage station 
            moveAuto(-0.8, 0, 0);// speed needs to be at least 0.8 for the robot to climb the charge station due to the arm 
        }
       else if (autoTime.get()<= 5){ // backwards onto the charge station until center 
            moveAuto(0.8, 0, 0);
        }
    
    }
    
    /*else if (mode==4){
        if(autoTime.get()<=2.4){// forwards 
            moveAuto(0.6,0,0);
        }
        if (autoTime.get()<=2.75){// left 
            moveAuto(0, -0.6, 0);
        }
        if  (autoTime.get()<=3){// backwards 
            moveAuto(-0.8, 0,0 );
        }
        
    }
    else if (mode == 5){
        if (autoTime.get()<=2.4){// forwards 
            moveAuto(0.6,0,0);
        }
    if  (autoTime.get()<=2.75){// right  
            moveAuto(0, 0.6, 0);
        }
    if  (autoTime.get()<=3){// backwards 
            moveAuto(-0.8, 0,0 );
     }
    }
    else if (mode == 6){
    if (autoTime.get()<=4){
            moveAuto(0.6, 0, 0);
        }
    }*/
    else if (mode == 7){
        if (autoTime.get()<= 15){
            moveAuto(0, 0, 0);
        }
    }
    else if (mode == 8){
        if  (autoTime.get()<=1){
            moveAuto(-0.6, 0, 0);
        }
     else   if (autoTime.get()<=3.5){
            moveAuto(0.6, 0, 0);
        }
    }
    else if (mode==9){
        if (autoTime.get()<=1){
            moveAuto(-0.6, 0, 0);
        }

    }

}

    
    


@Override
public boolean isFinished(){
    return false;

    
}
}


