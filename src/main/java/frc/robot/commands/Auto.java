package frc.robot.commands;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Gyroscope;


public class Auto extends CommandBase{
   public Auto() {
    addRequirements(RobotContainer.m_Drivetrain);
   } 
  
private void moveAuto(double y,double x,double z) {
RobotContainer.m_Drivetrain.driveCartesian(y,x,z,0); 
}
Timer autoTime = new Timer();
public int mode;
public static ADIS16470_IMU gyro = RobotContainer.m_imu;
double yAxisRate = 0;
 double zAxisRate = 0;
double outputz;
    final double allowedTurnAngle = 2;
      double stabilizingSpeed = 0.1;
  
      PIDController stablizePID = new PIDController(0.005, 0, 0.001);
      double correction = stablizePID.calculate(Gyroscope.gyro.getAngle());


@Override
public void initialize(){
    autoTime.reset();
}

@Override
public void execute() {
    
    SmartDashboard.putNumber("autoTime",autoTime.get());
      autoTime.start();

     if (mode == 2) {//forwards
        System.out.println("charge station only");
        if ( autoTime.get() <= 2.5){
            moveAuto(0.7,0,0);
        }
        else if(autoTime.get()<=2.6){
            if(Gyroscope.gyro.getAngle() > allowedTurnAngle){
                zAxisRate = -correction;
                SmartDashboard.putNumber("stabilizingSpeed", zAxisRate);
              }
           else if(Gyroscope.gyro.getAngle() < -allowedTurnAngle){
                zAxisRate = correction;
                SmartDashboard.putNumber("-stabilizingSpeed", zAxisRate);
              }
              else{
                zAxisRate = 0;
              }
         }
        else if(autoTime.get()<=15){
            moveAuto(0, 0, 0);
        }
    }
    
    
   /*  else if (mode == 3) {
        System.out.println("cube and charge station");
        if (autoTime.get()<= 1){// fowards over the chrage station 
            moveAuto(-0.8, 0, 0);// speed needs to be at least 0.8 for the robot to climb the charge station due to the arm 
        }
       else if (autoTime.get()<= 5){ // backwards onto the charge station until center 
            moveAuto(0.8, 0, 0);
        }
    
    }
    
    else if (mode==4){
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
    }*/
    else if (mode == 6){
    if (autoTime.get()<=3.5){
            moveAuto(0.6, 0, 0);
        }
    else if (autoTime.get()<=15){
            moveAuto(0, 0, 0);
        }
    }
    else if (mode == 7){ 
        if (autoTime.get()<= 15){
            moveAuto(0, 0, 0);
        }
    }
    else if (mode == 8){
        if  (autoTime.get()<=1){
            moveAuto(-0.6, 0, 0);
        }
     else   if (autoTime.get()<=3.2){
            moveAuto(0.6, 0, 0);
        }
     else if (autoTime.get()<=15){
            moveAuto(0, 0, 0);
        }

    }
    else if (mode==9){
        if (autoTime.get()<=1){
            moveAuto(-0.6, 0, 0);
        }
        else if (autoTime.get()<=15){
            moveAuto(0, 0, 0);
        }
    }
    else if (mode==10){// cube and charge station 
        if (autoTime.get()<=1){
            moveAuto(-0.6, 0, 0);
        }
        else if (autoTime.get()<=4){
           moveAuto(0.7, 0, 0);
           if(Gyroscope.gyro.getAngle() > allowedTurnAngle){
            zAxisRate = -correction;
            SmartDashboard.putNumber("stabilizingSpeed", zAxisRate);
          }

          if(Gyroscope.gyro.getAngle() < -allowedTurnAngle){
            zAxisRate = correction;
            SmartDashboard.putNumber("-stabilizingSpeed", zAxisRate);
          }
          else{
            zAxisRate = 0;
          }
        }
        else if (autoTime.get()<=7.4){
            moveAuto(0, 0, 0);
        }
        else if (autoTime.get()<=9.5){
           moveAuto(-0.7,0,0);
           if(Gyroscope.gyro.getAngle() > allowedTurnAngle){
            zAxisRate = -correction;
            SmartDashboard.putNumber("stabilizingSpeed", zAxisRate);
          }

          if(Gyroscope.gyro.getAngle() < -allowedTurnAngle){
            zAxisRate = correction;
            SmartDashboard.putNumber("-stabilizingSpeed", zAxisRate);
          }
          else{
            zAxisRate = 0;
          }
        }
        else if (autoTime.get()<=15){
            moveAuto(0, 0, 0);
        }
    }
    else if (mode == 11){//cube and forwards 2
        if  (autoTime.get()<=1){
            moveAuto(-0.6, 0, 0);
        }
     else   if (autoTime.get()<=3.4){
            moveAuto(0.6, 0, 0);
            
        }
    else if (autoTime.get()<=15){
            moveAuto(0, 0, 0);
        }
    }

}

    
    


@Override
public boolean isFinished(){
    return false;

    
}
}


