// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.EverybotIntake;

public class ControlIntake extends CommandBase {
  /** Creates a new ControlIntake. */
  private EverybotIntake everybotIntake;

  public ControlIntake(EverybotIntake everybotIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.everybotIntake = everybotIntake;
    addRequirements(everybotIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     int lastGamePiece = 0;
     final int CONE = 1;
     final int CUBE = 2;
     //final int NOTHING = 3;
     


    double intakePower;
    int intakeAmps;
    if (RobotContainer.xbox.getAButton()) {
    //if (RobotContainer.xboxSecond.getAButton()) {
      // cube in or cone out
      intakePower = EverybotIntake.IntakeOutputPower;
      intakeAmps = EverybotIntake.IntakeCurrentLimit;
      lastGamePiece = CUBE;
    } else if (RobotContainer.xbox.getBButton()) {
    //} else if (RobotContainer.xboxSecond.getBButton()) {
      // cone in or cube out
      intakePower = -EverybotIntake.IntakeOutputPower;
      intakeAmps = EverybotIntake.IntakeCurrentLimit;
      lastGamePiece = CONE;
    } else if (lastGamePiece == CUBE) {
      intakePower = EverybotIntake.IntakeHoldPower;
      intakeAmps = EverybotIntake.IntakeHoldCurrentLimit;
    } else if (lastGamePiece == CONE) {
      intakePower = -EverybotIntake.IntakeHoldPower;
      intakeAmps = EverybotIntake.IntakeHoldCurrentLimit;
    } else {
      intakePower = 0.0;
      intakeAmps = 0;
    }
    everybotIntake.setIntakeMotor(intakePower, intakeAmps);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
