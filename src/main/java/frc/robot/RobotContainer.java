// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ControlIntake;
import frc.robot.commands.MoveArm;
import frc.robot.commands.ArmCommands.MoveArmBottomRow;
import frc.robot.commands.ArmCommands.MoveArmMiddleRow;
import frc.robot.commands.ArmCommands.MoveArmStartingPosition;
import frc.robot.commands.ArmCommands.MoveArmTopRow;
import frc.robot.commands.ArmCommands.ResetEncoderTics;
import frc.robot.subsystems.EverybotArm;
import frc.robot.subsystems.EverybotIntake;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ButtonConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
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
  public static EverybotArm s_everybotArmSubsystem = new EverybotArm();
  public static EverybotIntake s_everybotIntakeSubsystem = new EverybotIntake();

  //Commands
  public static DriveRobot m_DriveRobot = new DriveRobot();
  public static MoveArmTopRow m_MoveArmTopRow = new MoveArmTopRow();
  public static MoveArmMiddleRow m_MoveArmMiddleRow = new MoveArmMiddleRow();
  public static MoveArmBottomRow m_MoveArmBottomRow = new MoveArmBottomRow();
  public static MoveArmStartingPosition m_MoveArmStartingPosition = new MoveArmStartingPosition();
  public static ResetEncoderTics m_ResetEncoderTics = new ResetEncoderTics();

  //Autonomous
  public static Auto m_autonomous = new Auto();

  //SendableChooser
  SendableChooser<Integer> autoChooser = new SendableChooser<>();
  public static SendableChooser<Boolean> DriveMode = new SendableChooser<>();
  public static SendableChooser<Boolean> Drivescheme = new SendableChooser<>();

  //OI
  //Controllers
  public static XboxController xbox = new XboxController(Constants.c_joystick);
  public static XboxController xboxSecond = new XboxController(Constants.c_joystickSecond);
  //Buttons
  public static JoystickButton armTopRowButton = new JoystickButton(xboxSecond, Constants.ButtonConstants.c_armTopRowButton);
  public static JoystickButton armMiddleRowButton = new JoystickButton(xboxSecond, Constants.ButtonConstants.c_armMiddleRowButton);
  public static JoystickButton armBottomRowButton = new JoystickButton(xboxSecond, Constants.ButtonConstants.c_armBottomRowButton);
  public static JoystickButton armStartingPositionButton = new JoystickButton(xboxSecond, Constants.ButtonConstants.c_armStartingPositionButton);
  public static JoystickButton resetArmEncoderTicsButton = new JoystickButton(xboxSecond, Constants.ButtonConstants.c_resetEncoderTicButton);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //set Default Commands for Subsystems
    s_everybotArmSubsystem.setDefaultCommand(new MoveArm(s_everybotArmSubsystem));
    s_everybotIntakeSubsystem.setDefaultCommand(new ControlIntake(s_everybotIntakeSubsystem));
    
    // Configure the button bindings
    configureButtonBindings();
    armTopRowButton.onTrue(m_MoveArmTopRow);
    armMiddleRowButton.onTrue(m_MoveArmMiddleRow);
    armBottomRowButton.onTrue(m_MoveArmBottomRow);
    armStartingPositionButton.onTrue(m_MoveArmStartingPosition);
    resetArmEncoderTicsButton.onTrue(m_ResetEncoderTics);
   
    //Chooser for DriverStation
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