// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ControlIntake;
import frc.robot.commands.MoveArm;
import frc.robot.commands.Autonomus.auto_CubeAndChargeStation;
import frc.robot.commands.Autonomus.auto_CubeAndForward;
import frc.robot.commands.Autonomus.auto_NoAuto;
import frc.robot.commands.Autonomus.auto_cubeAndStop;
import frc.robot.commands.Autonomus.auto_reverseforward;
import frc.robot.commands.Autonomus.automove;
import frc.robot.subsystems.EverybotArm;
import frc.robot.subsystems.EverybotIntake;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Gyroscope;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Auto;
import frc.robot.commands.DriveRobot;
import frc.robot.subsystems.Drivetrain;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //Subsystems
  public static Drivetrain m_Drivetrain = new Drivetrain();
  public static final ADIS16470_IMU m_imu = new ADIS16470_IMU();
  private EverybotArm s_everybotArmSubsystem = new EverybotArm();
  private EverybotIntake s_everybotIntakeSubsystem = new EverybotIntake();
  public static Gyroscope m_gyroscope = new Gyroscope();

  

  //Commands
  public static DriveRobot m_DriveRobot = new DriveRobot();
  public static Auto m_autonomous = new Auto();

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  public static SendableChooser<Boolean> DriveMode = new SendableChooser<>();
  public static SendableChooser<Boolean> Drivescheme = new SendableChooser<>();


  //OI
  public static XboxController xbox = new XboxController(Constants.c_joystick);
  public static XboxController xboxSecond = new XboxController(Constants.c_joystickSecond);
  public static JoystickButton armButton = new JoystickButton(xbox, 4);
  //getPOV can be used to find the ange value of the d-Pad on the xbox controller
  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_everybotArmSubsystem.setDefaultCommand(new MoveArm(s_everybotArmSubsystem));
    s_everybotIntakeSubsystem.setDefaultCommand(new ControlIntake(s_everybotIntakeSubsystem));
    
    SmartDashboard.putData(m_Drivetrain);
    // Configure the button bindings
    configureButtonBindings();
    //Is nessary, might have been the reason for the error "DifferntialDrive...Output not updated often enough"
    m_Drivetrain.setDefaultCommand(m_DriveRobot);
    autoChooser.addOption("out of community ONLY", new automove(.6, 0, 0, m_Drivetrain).withTimeout(2.5));
    autoChooser.addOption("cube and forward NON cable side", new auto_reverseforward(m_Drivetrain));
    autoChooser.addOption("cube and forwards cable side", new auto_CubeAndForward(m_Drivetrain));
    autoChooser.setDefaultOption("cube and stop", new auto_cubeAndStop(m_Drivetrain));
    autoChooser.addOption(" NO AUTO", new auto_NoAuto(m_Drivetrain));
    autoChooser.addOption("cube and Charge Station",  new auto_CubeAndChargeStation(m_Drivetrain));
    /* autoChooser.setDefaultOption("Cube and stop ", 9);
    autoChooser.addOption("Charge station ONLY", 2);
    autoChooser.addOption( "out of comuntity ONLY", 6);
    autoChooser.addOption("NO AUTO", 7);
    autoChooser.addOption("Cube  and forwards NON cable sides", 8);
    autoChooser.addOption("Cube, leave and charge station ONLY", 10);//DO NOT USE UNLESS YOU WILL BE GOING ON THE CHARGESTATION
    autoChooser.addOption("cube and forwards cable side  ", 11); */
    Drivescheme.setDefaultOption("Katelyn", true);
    Drivescheme.addOption("Matthew", false);
    SmartDashboard.putData("Autonomous Mode", autoChooser);
    SmartDashboard.putData("driver", Drivescheme);
    DriveMode.setDefaultOption("Field Oriented", true);
    DriveMode.addOption("Robot Oriented", false);
    SmartDashboard.putData("Drive Mode", DriveMode);


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}