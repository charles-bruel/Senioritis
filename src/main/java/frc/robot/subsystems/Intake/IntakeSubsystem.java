package frc.robot.subsystems.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private IntakeIO io;
  private double voltage = Constants.IntakeConstants.IDLE_VOLTAGE;

  public IntakeSubsystem(IntakeIO intakeIO) {
    io = intakeIO;
  }

  public void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12, 12);
    this.voltage = voltage;
  }

  @Override
  public void periodic() {
    io.setVoltage(voltage);
    Logger.getInstance().recordOutput("Intake/Output", voltage);
  }

  public static class Commands {
    public static Command setVoltage(double volts) {
      return new InstantCommand(() -> Robot.intake.setVoltage(volts));
    }
  }
}
