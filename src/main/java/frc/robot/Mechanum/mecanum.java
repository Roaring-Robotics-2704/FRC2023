package frc.robot.Mechanum;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Mechanum.mechanumConf.wheelLocations;


public class mecanum{
    // Locations of the wheels relative to the robot center.

static Translation2d m_frontLeftLocation = new Translation2d(wheelLocations.frontLeftWheelX, wheelLocations.frontLeftWheelY);
static Translation2d m_frontRightLocation = new Translation2d(wheelLocations.frontRightWheelX, wheelLocations.frontRightWheelY);
static Translation2d m_backLeftLocation = new Translation2d(wheelLocations.backLeftWheelX, wheelLocations.backLeftWheelY);
static Translation2d m_backRightLocation = new Translation2d(wheelLocations.backRightWheelX, wheelLocations.backRightWheelY);

// Creating my kinematics object using the wheel locations.
static MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
);

public static ChassisSpeeds createSpeeds(double x,double y,double z) {
    ChassisSpeeds speeds = new ChassisSpeeds(y,x,z);
    return speeds;
}
public static MecanumDriveWheelSpeeds calculateSpeeds(ChassisSpeeds speeds) {
    MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
    return wheelSpeeds;
}
public static void veloDrive(
    MecanumDriveWheelSpeeds speed,
    WPI_TalonSRX frontleftSrx, 
    WPI_TalonSRX frontrightSrx,
    WPI_TalonSRX backleftSrx,
    WPI_TalonSRX backrightSrx,
    MecanumDrive drive
) {
    frontleftSrx.set(ControlMode.Velocity,speed.frontLeftMetersPerSecond);
    frontrightSrx.set(ControlMode.Velocity, speed.frontRightMetersPerSecond);
    backleftSrx.set(ControlMode.Velocity, speed.rearLeftMetersPerSecond);
    backrightSrx.set(ControlMode.Velocity, speed.rearRightMetersPerSecond);
    drive.feed();
}
}
