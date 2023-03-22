package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
@AutoLog
public class EverybotIntake extends SubsystemBase {
  /** Creates a new EverybotIntake. */
  //Create new CANSparkMax
  CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeConstants.c_intakeMotor, MotorType.kBrushless);

  /*public static final int IntakeCurrentLimit = Constants.IntakeConstants.c_intakeCurrentLimit; //Amps can use when picking up
  public static final int IntakeHoldCurrentLimit = Constants.IntakeConstants.c_intakeHoldCurrentLimit; //Amps can use when holding
  
  public static final double IntakeOutputPower = Constants.IntakeConstants.c_intakeOutputPower; //Precent output for intaking
  public static final double IntakeHoldPower = Constants.IntakeConstants.c_intakeHoldPower; //Precent output for holding
*/
  public EverybotIntake() {}

  public void setIntakeMotor(double percent, int amps) {
    intakeMotor.set(percent);
    intakeMotor.setSmartCurrentLimit(amps);
    SmartDashboard.putNumber("intake power (%)", percent);
    SmartDashboard.putNumber("intake motor current (amps)", intakeMotor.getOutputCurrent());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}