package frc.robot.subsystems.Pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public class PivotInputs {
    public double motorEncoderAngle;
    public double absoluteEncoderAngle;

    public double motorRightOutputVolts;
    public double motorRightOutputAmpsSupply;
    public double motorRightOutputAmpsStator;
    public double motorLeftOutputVolts;
    public double motorLeftOutputAmpsSupply;
    public double motorLeftOutputAmpsStator;
  }

  public void updateInputs(PivotInputs inputs);

  public void setVoltage(double volts);

  public void seed(PivotInputs inputs);
}
