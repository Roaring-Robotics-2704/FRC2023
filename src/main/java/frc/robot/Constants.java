package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int c_joystick = 0;
  }

  public static class ArmConstants{
    public static final int c_armMotor = 5;//Motor port
    public static final double c_armPowerIn = 0.35;//Precent output when go up and down
    public static final double c_armPowerOut = 0.4;
    public static final double c_armEncoderKp = 0;
    public static final double c_armEncoderKi = 0;
    public static final double c_armEncoderKd = 0;
    public static final double c_topRow = 10;
    public static final double c_middleRow = 5;
    public static final double c_startinPosition = 0;
  }

  public static class IntakeConstants{
    public static final int c_intakeMotor = 6;//Motor port
    public static final int c_intakeCurrentLimit = 25;//Amps can use when picking up
    public static final int c_intakeHoldCurrentLimit = 5;//Amps can use when holding
    public static final double c_intakeOutputPower = 1.0;//Precent output for intaking
    public static final double c_intakeOutputPowerSlower = 0.20;
    public static final double c_intakeHoldPower = 0.07;//Precent output for holding
  }
  public static class GyroConstants{
    public static final double c_MaximumAllowedAngle = 2.5; //is the maximum allowed in order to be consider engaged and docked(level)
    public static final double c_TargetAngle  = 2; //this number might need to be changed, it is when we stop adjusting
  }


    public static int c_joystick = 0;
    public static int c_joystickSecond = 1;

    public static int c_frontleftDriveMotor = 4;
    public static int c_frontrightDriveMotor = 3;
    public static int c_backleftDriveMotor = 2;
    public static int c_backrightDriveMotor = 1;

    public static int c_leftJoystickAxisx = 1;
    public static int c_rightJoystickAxisx = 5;
    public static int c_rightJoystickAxisy = 5;
    public final static class zpid {
        public static double p = 0.005;
        public static double i = 0;
        public static double d = 0;
    }
    public static int setup = 10;
    public static double c_speedcap = 0.5;//0.75 speed is good. y of xbox times this number.
}
