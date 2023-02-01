// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class OdometryKinematicDriveTrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

    public static final double maxSpeed = 3.0;
    public static final double maxAngularSpeed = Math.PI;

    //Declare Motor Controllers 
    //VictorSPX
    private WPI_VictorSPX m_frontLeft = new WPI_VictorSPX(Constants.DriveTrain.c_frontLeftMotor);
    private WPI_VictorSPX m_backLeft = new WPI_VictorSPX(Constants.DriveTrain.c_backLeftMotor);

    //TalonSRXs
    //private WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(Constants.DriveTrain.c_frontLeftMotor);
    //private WPI_TalonSRX m_backLeft = new WPI_TalonSRX(Constants.DriveTrain.c_backLeftMotor);

    private WPI_TalonSRX m_frontRight = new WPI_TalonSRX(Constants.DriveTrain.c_frontRightMotor);
    private WPI_TalonSRX m_backRight = new WPI_TalonSRX(Constants.DriveTrain.c_backRightMotor);

    //Decalre Encoders
    private Encoder e_frontLeftEncoder = new Encoder(0,1);
    private Encoder e_backLeftEncoder = new Encoder(2,3);
    private Encoder e_frontRightEncoder = new Encoder(4,5);
    private Encoder e_backRightEncoder = new Encoder(5,6);

    private Translation2d l_frontLeftLocation = new Translation2d(0.381, 0.381);
    private Translation2d l_backLeftLocation = new Translation2d(-0.381, 0.381);
    private Translation2d l_frontRightLocation = new Translation2d(0.381, -0.381);
    private Translation2d l_backRightLocation = new Translation2d(-0.381, -0.381);

    private PIDController c_frontLeftPIDController = new PIDController(1, 0, 0);
    private PIDController c_backLeftPIDController = new PIDController(1, 0, 0);
    private PIDController c_frontRightPIDController = new PIDController(1, 0, 0);
    private PIDController c_backRightPIDController = new PIDController(1, 0, 0);

    private MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(l_frontLeftLocation, l_frontRightLocation, l_backLeftLocation, l_backRightLocation);

    private MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(m_kinematics, Gyroscope.gyro.getAngle(), getCurrentDistances());

    //will need to calculate gains based on robot
    private SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(1, 3);

  public OdometryKinematicDriveTrain() {
    Gyroscope.gyro.reset();
    //in here might have to invert motors so that when given a postivie voltage both side move foward
  }

  public MecanumDriveWheelSpeeds getCurrentState(){
    return new MecanumDriveWheelSpeeds(e_frontLeftEncoder.getRate(), e_frontRightEncoder.getRate(), e_backLeftEncoder.getRate(), e_backRightEncoder.getRate());
  }

  public MecanumDriveWheelPositions getCurrentDistance(){
    return new MecanumDriveWheelPositions(e_frontLeftEncoder.getDistance(), e_frontRightEncoder.getDistance(), e_backLeftEncoder.getDistance(), e_backRightEncoder.getDistance());
  }

  public void setSpeeds(MecanumDriveWheelSpeeds speeds){
    final double frontLeftFeedforward = m_feedForward.calculate(speeds.frontLeftMetersPerSecond);
    final double frontRightFeedforward = m_feedForward.calculate(speeds.frontRightMetersPerSecond);
    final double backLeftFeedforward = m_feedForward.calculate(speeds.rearLeftMetersPerSecond);
    final double backRightFeedforward = m_feedForward.calculate(speeds.rearRightMetersPerSecond);

    final double frontLeftOutput = c_frontLeftPIDController.calculate( e_frontLeftEncoder.getRate(), speeds.frontLeftMetersPerSecond);
    final double frontRightOutput = c_frontRightPIDController.calculate( e_frontRightEncoder.getRate(), speeds.frontRightMetersPerSecond);
    final double backLeftOutput = c_backLeftPIDController.calculate(e_backLeftEncoder.getRate(), speeds.rearLeftMetersPerSecond);
    final double backRightOutput = c_backRightPIDController.calculate(e_backRightEncoder.getRate(), speeds.rearRightMetersPerSecond);

    m_frontLeft.setVoltage(frontLeftOutput + frontLeftFeedforward);
    m_frontRight.setVoltage(frontRightOutput + frontRightFeedforward);
    m_backLeft.setVoltage(backLeftOutput + backLeftFeedforward);
    m_backRight.setVoltage(backRightOutput + backRightFeedforward);
  }

  public void drive(double xMotorSpeed, double yMotorSpeed, double zMotorSpeed, boolean fieldRelative){
    var MecanumDriveWheelSpeeds = m_kinematics.toWheelSpeeds(fieldRelative?ChassisSpeeds.fromFieldRelativeSpeeds(xMotorSpeed, yMotorSpeed, zMotorSpeed, Gyroscope.gyro.getAngle()): new ChassisSpeeds(xMotorSpeed, yMotorSpeed, zMotorSpeed));
    MecanumDriveWheelSpeeds.desaturate(maxSpeed);
    setSpeeds(MecanumDriveWheelSpeeds);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
