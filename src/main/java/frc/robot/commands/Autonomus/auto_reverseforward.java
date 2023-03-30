// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomus;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class auto_reverseforward extends SequentialCommandGroup {
  /** Creates a new auto_reverseforward. */
  public auto_reverseforward(Drivetrain m_Drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( new automove(-0.6, 0, 0, m_Drivetrain).withTimeout(1),
                    new WaitCommand(.1),
                    new automove(0.6, 0, 0, m_Drivetrain).withTimeout(3.25));
  }
}
