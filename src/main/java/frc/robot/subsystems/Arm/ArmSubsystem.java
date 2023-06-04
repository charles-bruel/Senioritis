package frc.robot.subsystems.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Robot;
import frc.robot.utilities.PIDFFController;
import frc.robot.utilities.SuperstructureConfig;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {

  private ArmIO io;
  private ArmInputsAutoLogged inputs;
  private final PIDFFController controller;
  private double targetHeight = 0;

  public ArmSubsystem(ArmIO armIO) {
    io = armIO;
    controller = new PIDFFController(Constants.ArmConstants.GAINS);
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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    double output = controller.calculate(inputs.absoluteEncoderHeight, targetHeight);
    output = Robot.operator.getLeftY() * 12;
    io.setVoltage(output);

    output = MathUtil.clamp(output, -12, 12);
    Logger.getInstance().recordOutput("Arm/Target Height", targetHeight);
    Logger.getInstance().recordOutput("Arm/Output", output);

    Logger.getInstance().processInputs("Arm", inputs);
  }

  public static class Commands {
    public static Command setHeight(double targetHeight) {
      return new InstantCommand(() -> Robot.arm.setTargetHeight(targetHeight), Robot.arm);
    }

    public static Command setHeight(SuperstructureConfig config) {
      return setHeight(config.getArmHeight());
    }

    public static Command setHeightBlocking(double angle) {
      return edu.wpi.first.wpilibj2.command.Commands.run(
              () -> Robot.arm.setTargetHeight(angle), Robot.arm)
          .until(Robot.arm::isAtTarget);
    }

    public static Command setHeightBlocking(SuperstructureConfig config) {
      return setHeightBlocking(config.getArmHeight());
    }
  }
}
