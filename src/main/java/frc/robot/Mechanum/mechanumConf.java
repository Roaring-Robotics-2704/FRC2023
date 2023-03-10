package frc.robot.Mechanum;

public class mechanumConf extends mecanum {
    public static class wheelLocations {
        //front left wheel offset to center of bot in meters
        public static double frontLeftWheelX = 0.381; //+ is to front of robot
        public static double frontLeftWheelY = 0.381; //+ is to left of robot
        
        //left wheel offset to center of bot in meters
        public static double frontRightWheelX = 0.381; //+ is to front of robot
        public static double frontRightWheelY = -0.381; //+ is to left of robot
       
        //front left wheel offset to center of bot in meters
        public static double backLeftWheelX = -0.381; //+ is to front of robot
        public static double backLeftWheelY = 0.381; //+ is to left of robot
        
        //left wheel offset to center of bot in meters
        public static double backRightWheelX = -0.381; //+ is to front of robot
        public static double backRightWheelY = -0.381; //+ is to left of robot
    } 

}
