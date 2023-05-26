package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.Constants;

public class IntakeIOSparkMAXPWN implements IntakeIO {
  private final PWMSparkMax intakeMotor;

  public IntakeIOSparkMAXPWN() {
    intakeMotor = new PWMSparkMax(Constants.RobotMap.INTAKE);
  }

  @Override
  public void setVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }
}
