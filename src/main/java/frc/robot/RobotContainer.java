// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
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
  String trajectoryJSON = "paths/YourPath.wpilib.json";
  Trajectory trajectory = new Trajectory();

  //Commands
  public static DriveRobot m_DriveRobot = new DriveRobot();
  public static Auto m_autonomous = new Auto();

  SendableChooser<Integer> autoChooser = new SendableChooser<>();
  public static SendableChooser<Boolean> DriveMode = new SendableChooser<>();
  public static SendableChooser<Boolean> Drivescheme = new SendableChooser<>();
  //OI
  public static XboxController xbox = new XboxController(Constants.c_joystick);
  //getPOV can be used to find the ange value of the d-Pad on the xbox controller


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //Is nessary, might have been the reason for the error "DifferntialDrive...Output not updated often enough"
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
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
   }

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

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
