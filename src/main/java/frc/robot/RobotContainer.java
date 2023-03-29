// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.GyroDrive;
import frc.robot.subsystems.Drivetrain;
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
  public static Gyroscope m_gyroscope = new Gyroscope();
  //Subsystems
  public static Drivetrain m_Drivetrain = new Drivetrain();
 // public static final ADIS16470_IMU m_imu = new ADIS16470_IMU();


  //Commands
  public static DriveRobot m_DriveRobot = new DriveRobot();
  public static Auto m_autonomous = new Auto();

  //SendableChooser
  SendableChooser<Integer> autoChooser = new SendableChooser<>();
  public static SendableChooser<Boolean> DriveMode = new SendableChooser<>();
  public static SendableChooser<Boolean> Drivescheme = new SendableChooser<>();

  //OI
  public static XboxController xbox = new XboxController(Constants.c_joystick);
  public static JoystickButton gryoButton = new JoystickButton(xbox, 3);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //Gyroscope
    m_Drivetrain.setDefaultCommand(new GyroDrive(m_Drivetrain, m_gyroscope));

    //Drivetrain
    m_Drivetrain.setDefaultCommand(m_DriveRobot);
    autoChooser.setDefaultOption("square", 1);
    autoChooser.addOption("Back up", 2);
    autoChooser.addOption("probably chaos",3);
    autoChooser.addOption("chaos square",4);
    autoChooser.addOption("self align",5);
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
    //starts command when condition changes to true and cancels when the condition changes to false
    //toggles gyrodrive on and off
    gryoButton.whileTrue(new GyroDrive(m_Drivetrain, m_gyroscope));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_autonomous.mode = autoChooser.getSelected();
    return m_autonomous;
  }
}
