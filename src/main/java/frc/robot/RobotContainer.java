// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

//import frc.robot.commands.DriveToTargetDistance;
//import frc.robot.commands.RotateToTargetYaw;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
 
  public static Drivetrain m_drivetrain = new Drivetrain();
  public static Vision m_vision = new Vision();
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static XboxController m_xbox =
      new XboxController(0);
  public static JoystickButton distanceButton = new JoystickButton(m_xbox, 4);
  public static JoystickButton rotateButton = new JoystickButton(m_xbox, 1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    //m_drivetrain.setDefaultCommand(new DefaultDrive(m_drivetrain, m_vision, m_xbox));
    m_drivetrain.setDefaultCommand(new OnlyDriving(m_drivetrain, m_xbox));
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    //distanceButton.onTrue(new DriveToDistance(m_drivetrain, m_vision, m_xbox).withTimeout(4).andThen(new TurnToAngle(m_drivetrain, m_vision, m_xbox).withTimeout(2)));
    distanceButton.onTrue(new DriveToDistance(m_drivetrain, m_vision, m_xbox).andThen(new TurnToAngle(m_drivetrain, m_vision, m_xbox)));
    rotateButton.onTrue(new TurnToAngle(m_drivetrain, m_vision, m_xbox));

  }

}
