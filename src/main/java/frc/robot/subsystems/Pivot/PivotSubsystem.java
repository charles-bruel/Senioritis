package frc.robot.subsystems.Pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Robot;
import frc.robot.utilities.PIDFFController;
import frc.robot.utilities.SuperstructureConfig;
import org.littletonrobotics.junction.Logger;

public class PivotSubsystem extends SubsystemBase {

  private PivotIO io;
  private PivotInputsAutoLogged inputs;
  private final PIDFFController controller;
  private double targetAngle = 90;

  public PivotSubsystem(PivotIO pivotIO) {
    io = pivotIO;
    controller = new PIDFFController(PivotConstants.GAINS);
    inputs = new PivotInputsAutoLogged();
    io.updateInputs(inputs);
  }

  public boolean isAtTarget() {
    return Math.abs(targetAngle - inputs.absoluteEncoderAngle) < PivotConstants.EPSILON;
  }

  public void setTargetAngle(double newAngle) {
    // if (newAngle < PivotConstants.MIN_ANGLE || newAngle > PivotConstants.MAX_ANGLE) {
    //   return;
    // }
    targetAngle = newAngle;
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    double output = controller.calculate(inputs.absoluteEncoderAngle, targetAngle);
    output =
        MathUtil.clamp(output, -PivotConstants.MAX_OUTPUT_VOLTS, PivotConstants.MAX_OUTPUT_VOLTS);
    io.setVoltage(output);

    Logger.getInstance().recordOutput("Pivot/Target Angle", targetAngle);
    Logger.getInstance().recordOutput("Pivot/Output", output);

    Logger.getInstance().processInputs("Pivot", inputs);
  }

  public static class Commands {
    public static Command setPosition(double angle) {
      return new InstantCommand(() -> Robot.pivot.setTargetAngle(angle));
    }

    public static Command setPosition(SuperstructureConfig config) {
      return setPosition(config.getPivotPosition());
    }

    public static Command setPositionBlocking(double angle) {
      return edu.wpi.first.wpilibj2.command.Commands.run(
              () -> Robot.pivot.setTargetAngle(angle), Robot.pivot)
          .until(Robot.pivot::isAtTarget);
    }

    public static Command setPositionBlocking(SuperstructureConfig config) {
      return setPositionBlocking(config.getPivotPosition());
    }
  }
}
