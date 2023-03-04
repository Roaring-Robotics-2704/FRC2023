
/*package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.EverybotArm;

public class MoveArm extends CommandBase {
  /** Creates a new MoveArm. */
  /*private EverybotArm everybotArm;
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
    if (RobotContainer.xbox.getXButton()) {
    //if (RobotContainer.xboxSecond.getXButton()) {
      // lower the arm
      armPower = -EverybotArm.ArmOutputPower;
    } else if (RobotContainer.xbox.getYButton()) {
    //} else if (RobotContainer.xboxSecond.getYButton()) {
      // raise the arm
      armPower = EverybotArm.ArmOutputPower;
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
}*/