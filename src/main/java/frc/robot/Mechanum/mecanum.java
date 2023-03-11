package frc.robot.Mechanum;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Mechanum.mechanumConf.motorPorts;
import frc.robot.Mechanum.mechanumConf.wheelLocations;


public class mecanum{
    // Locations of the wheels relative to the robot center.
static WPI_TalonSRX frontleftSrx = new WPI_TalonSRX(motorPorts.frontleft);
static WPI_TalonSRX frontrightSrx = new WPI_TalonSRX(motorPorts.frontright);
static WPI_TalonSRX backleftSrx = new WPI_TalonSRX(motorPorts.backleft);
static WPI_TalonSRX backrightSrx = new WPI_TalonSRX(motorPorts.backright);
static MecanumDrive drive = new MecanumDrive(frontleftSrx, frontrightSrx, backleftSrx, backrightSrx);

ErrorCode fl = frontleftSrx.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
ErrorCode fr = frontrightSrx.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
ErrorCode bl = backleftSrx.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
ErrorCode br = backrightSrx.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);



static Translation2d m_frontLeftLocation = new Translation2d(wheelLocations.frontLeftWheelX, wheelLocations.frontLeftWheelY);
static Translation2d m_frontRightLocation = new Translation2d(wheelLocations.frontRightWheelX, wheelLocations.frontRightWheelY);
static Translation2d m_backLeftLocation = new Translation2d(wheelLocations.backLeftWheelX, wheelLocations.backLeftWheelY);
static Translation2d m_backRightLocation = new Translation2d(wheelLocations.backRightWheelX, wheelLocations.backRightWheelY);

// Creating my kinematics object using the wheel locations.
static MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
);

public static ChassisSpeeds createSpeeds(double x,double y,double z) {
    ChassisSpeeds speeds = new ChassisSpeeds(y,x,Math.toDegrees(z));
    return speeds;
}
public static MecanumDriveWheelSpeeds calculateSpeeds(ChassisSpeeds speeds) {
    MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
    return wheelSpeeds;
}
public static void veloDrive(double y,double x,double z) {
    MecanumDriveWheelSpeeds speed = calculateSpeeds(createSpeeds(y, x, z));
    frontleftSrx.set(ControlMode.Velocity,speed.frontLeftMetersPerSecond);
    frontrightSrx.set(ControlMode.Velocity, speed.frontRightMetersPerSecond);
    backleftSrx.set(ControlMode.Velocity, speed.rearLeftMetersPerSecond);
    backrightSrx.set(ControlMode.Velocity, speed.rearRightMetersPerSecond);
    drive.feed();    
    
    SmartDashboard.putNumber("frontleftV", frontleftSrx.getSelectedSensorVelocity());
    SmartDashboard.putNumber("frontrightV", frontrightSrx.getSelectedSensorVelocity());
    SmartDashboard.putNumber("backleftV", backleftSrx.getSelectedSensorVelocity());
    SmartDashboard.putNumber("backrightV", backrightSrx.getSelectedSensorVelocity());

}
public static void driveCartesian(double y,double x,double z) {
    drive.driveCartesian(y, x, z);
}
}
