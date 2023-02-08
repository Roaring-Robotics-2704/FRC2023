// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.EverybotArm;

public class MoveArm extends CommandBase {
  /** Creates a new MoveArm. */
  private EverybotArm everybotArm;
  public MoveArm(EverybotArm everybotArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.everybotArm = everybotArm;
    addRequirements(everybotArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double armPower;
    if (RobotContainer.m_driverJoystick.getRawButton(7)) {
      // lower the arm
      armPower = -everybotArm.ArmOutputPower;
    } else if (RobotContainer.m_driverJoystick.getRawButton(5)) {
      // raise the arm
      armPower = everybotArm.ArmOutputPower;
    } else {
      // do nothing and let it sit where it is
      armPower = 0.0;
    }
    everybotArm.setArmMotor(armPower);

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
