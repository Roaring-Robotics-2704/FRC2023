/*package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EverybotArm extends SubsystemBase {
  /** Creates a new EverybotArm. */
  //Create new CANSparkMax
 /*public TalonSRX armMotor = new TalonSRX(Constants.ArmConstants.c_armMotor); 

  public static final int ArmCurrentLimit = Constants.ArmConstants.c_armCurrentLimit; //Amps motor can use
  public static final double ArmOutputPower = Constants.ArmConstants.c_armOutputPower; //Precent output when go up and down

  public EverybotArm() {}

  public void setArmMotor(double percent) {
    armMotor.set(TalonSRXControlMode.PercentOutput, percent);
    SmartDashboard.putNumber("arm power (%)", percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}*/