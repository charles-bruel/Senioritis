package frc.robot.subsystems.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Robot;
import frc.robot.utilities.PIDFFController;
import frc.robot.utilities.RotatingElevatorFeedforward;
import frc.robot.utilities.SuperstructureConfig;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {

  private ArmIO io;
  private ArmInputsAutoLogged inputs;
  private final PIDFFController controller;
  private final RotatingElevatorFeedforward feedforward;
  private double lastPosition;
  private double targetHeight = 15;

  public ArmSubsystem(ArmIO armIO) {
    io = armIO;
    controller = new PIDFFController(ArmConstants.GAINS);
    feedforward = ArmConstants.GAINS.createRotatingElevatorFeedforward();
    inputs = new ArmInputsAutoLogged();
    io.updateInputs(inputs);
  }

  public void setTargetHeight(double newHeight) {
    if (newHeight < ArmConstants.MIN_HEIGHT || newHeight > ArmConstants.MAX_HEIGHT) {
      return;
    } else targetHeight = newHeight;
  }

  public boolean isAtTarget() {
    return Math.abs(targetHeight - inputs.motorEncoderHeight) < ArmConstants.EPSILON;
  }

  public void dontMove() {
    targetHeight = inputs.absoluteEncoderHeight;
  }

  @Override
  public void periodic() {
    double velocity = (inputs.absoluteEncoderHeight - lastPosition) / 0.02;
    io.updateInputs(inputs);
    double pidValue = controller.calculate(inputs.absoluteEncoderHeight, targetHeight);
    pidValue =
        MathUtil.clamp(
            pidValue, -ArmConstants.MAX_PID_OUTPUT_VOLTS, ArmConstants.MAX_PID_OUTPUT_VOLTS);
    // Bad hack since we dont use kv
    double feedforwardValue =
        feedforward.calculate(
            Robot.pivot.getCurrentAngle(), targetHeight - inputs.absoluteEncoderHeight);
    double output = pidValue + feedforwardValue;

    // Code to create a good way to create setpoints
    double v = Robot.operator.getRightY();
    v = MathUtil.applyDeadband(v, 0.2);
    if (v != 0) {
      output = v * 12;
      targetHeight = inputs.absoluteEncoderAngle;
    }
    output = MathUtil.clamp(output, -ArmConstants.MAX_OUTPUT_VOLTS, ArmConstants.MAX_OUTPUT_VOLTS);

    io.setVoltage(output);

    Logger.getInstance().recordOutput("Arm/Target Height", targetHeight);
    Logger.getInstance().recordOutput("Arm/Output", output);
    Logger.getInstance().recordOutput("Arm/PIDOutput", pidValue);
    Logger.getInstance().recordOutput("Arm/FFOutput", feedforwardValue);
    Logger.getInstance().recordOutput("Arm/Velocity", velocity);

    Logger.getInstance().processInputs("Arm", inputs);
    lastPosition = inputs.absoluteEncoderHeight;
  }

  public static class Commands {
    public static Command setHeight(double targetHeight) {
      return new InstantCommand(() -> Robot.arm.setTargetHeight(targetHeight), Robot.arm);
    }

    public static Command setHeight(SuperstructureConfig config) {
      return setHeight(config.getArmHeight());
    }

    public static Command setHeightAndWait(double angle) {
      return new RunCommand(() -> setHeight(angle), Robot.arm).until(Robot.arm::isAtTarget);
    }

    public static Command setHeightAndWait(SuperstructureConfig config) {
      return setHeightAndWait(config.getArmHeight());
    }
  }
}
