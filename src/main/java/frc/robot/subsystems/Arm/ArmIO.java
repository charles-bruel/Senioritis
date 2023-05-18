package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public class ArmInputs {
    public double motorEncoderHeight = 0;
    public double absoluteEncoderHeight = 0;

    public double motorOutputVolts = 0;
    public double motorOutputAmps = 0;
  }

  public void updateInputs(ArmInputs inputs);

  public void setVoltage(double volts);
}
