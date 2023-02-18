package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants;



public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  //Drive Train Motors
  private WPI_TalonSRX m_frontrightMotor = new WPI_TalonSRX(Constants.c_frontrightDriveMotor);
  
  private WPI_TalonSRX m_backrightMotor = new WPI_TalonSRX(Constants.c_backrightDriveMotor);
  
  
  private WPI_TalonSRX m_frontleftMotor = new WPI_TalonSRX(Constants.c_frontleftDriveMotor);
  
  private WPI_TalonSRX m_backleftMotor = new WPI_TalonSRX(Constants.c_backleftDriveMotor);

  private MecanumDrive drive = new MecanumDrive(m_frontleftMotor, m_backleftMotor, m_frontrightMotor, m_backrightMotor);
  MotorControllerGroup m_right = new MotorControllerGroup(m_frontrightMotor, m_backrightMotor);
  MotorControllerGroup m_left = new MotorControllerGroup(m_frontleftMotor, m_backleftMotor);
// encoders
// yes,this is as painful as it seems 
   ErrorCode frontRightEncoder = m_frontrightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    double frontRightEncoderDistance = m_frontrightMotor.getSelectedSensorPosition();//basically a verison of the get.distance()from the wpilib encoder class.
    double frontRightEncoderRate = m_frontrightMotor.getSelectedSensorVelocity();//basically a verison of the get.Rate()from the wpilib encoder class.
   ErrorCode frontLeftEncoder = m_frontleftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
     double frontLeftEncoderDistance = m_frontleftMotor.getSelectedSensorPosition();
     double frontLeftEncoderRate = m_frontleftMotor.getSelectedSensorVelocity();
   ErrorCode backRightEncoder = m_backrightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
     double backRightEncoderDistance = m_backrightMotor.getSelectedSensorPosition();
     double backRightEncoderRate = m_backleftMotor.getSelectedSensorVelocity();
   ErrorCode backLeftEncoder = m_backleftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
     double backLeftEncoderDistance = m_backleftMotor.getSelectedSensorPosition();
     double backLeftEncoderRate = m_backleftMotor.getSelectedSensorVelocity();

    
/*  the first line with ErrorCode is setting the encoder to a motor. 
    the first double line sets it to a verion of the get.distance in the wpilb encoder class 
    the second double line in each sets it to a version  of get.Rate() from the wpilib encoder class */ 
  public void driveCartesian(double y, double x, double z,double rotation){
    Rotation2d heading = Rotation2d.fromDegrees(rotation);
    drive.driveCartesian(-y,x,-z,heading);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}