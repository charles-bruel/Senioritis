package frc.robot.subsystems.Pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.utilities.PIDFFController;

public class PivotSubsystem extends SubsystemBase {

  private PivotIO io;
  private PivotInputsAutoLogged inputs;
  private final PIDFFController controller;
  private double targetAngle = 0;

  public PivotSubsystem(PivotIO pivotIO) {
    io = pivotIO;
    controller = new PIDFFController(PivotConstants.GAINS);
    inputs = new PivotInputsAutoLogged();
    io.updateInputs(inputs);
  }

  public void setTargetAngle(double newAngle) {
    if (newAngle < PivotConstants.MIN_ANGLE || newAngle > PivotConstants.MAX_ANGLE) {
      return;
    }
    targetAngle = newAngle;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    double output = controller.calculate(inputs.absoluteEncoderAngle, targetAngle);
    io.setVoltage(output);

    output = MathUtil.clamp(output, -12, 12);
    Logger.getInstance().recordOutput("Pivot/Target Angle", targetAngle);
    Logger.getInstance().recordOutput("Pivot/Output", output);

    Logger.getInstance().processInputs("Pivot", inputs);
  }
}
