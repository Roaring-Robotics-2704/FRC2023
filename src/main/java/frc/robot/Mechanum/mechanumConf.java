package frc.robot.Mechanum;

public class mechanumConf extends mecanum {
    public static class wheelLocations {
        //front left wheel offset to center of bot in meters
        public static double frontLeftWheelX = 0.2286; //+ is to front of robot
        public static double frontLeftWheelY = 0.2286; //+ is to left of robot
        
        //left wheel offset to center of bot in meters
        public static double frontRightWheelX = 0.2286; //+ is to front of robot
        public static double frontRightWheelY = -0.2286; //+ is to left of robot
       
        //front left wheel offset to center of bot in meters
        public static double backLeftWheelX = -0.2286; //+ is to front of robot
        public static double backLeftWheelY = 0.2286; //+ is to left of robot
        
        //left wheel offset to center of bot in meters
        public static double backRightWheelX = -0.2286; //+ is to front of robot
        public static double backRightWheelY = -0.2286; //+ is to left of robot
    } 
    public static class motorPorts {
        //motor ports on can bus
        public static int frontleft = 1;
        public static int frontright = 1;
        public static int backleft = 1;
        public static int backright = 1;
    }
    public static class anglepid {
        public static double p = 0;
        public static double i = 0;
        public static double d = 0;
    }
}
