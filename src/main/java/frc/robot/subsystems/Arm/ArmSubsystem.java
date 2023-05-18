package frc.robot.subsystems.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.utilities.PIDFFController;
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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    double output = controller.calculate(inputs.absoluteEncoderHeight, targetHeight);
    io.setVoltage(output);

    output = MathUtil.clamp(output, -12, 12);
    Logger.getInstance().recordOutput("Arm/Target Height", targetHeight);
    Logger.getInstance().recordOutput("Arm/Output", output);

    Logger.getInstance().processInputs("Arm", inputs);
  }
}
