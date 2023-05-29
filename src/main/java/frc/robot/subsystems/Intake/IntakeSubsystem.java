package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utilities.SuperstructureConfig;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private IntakeIO io;

  public IntakeSubsystem(IntakeIO intakeIO) {
    io = intakeIO;
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
    Logger.getInstance().recordOutput("Intake/Output", voltage);
  }

  public static class Commands {
    public static Command setVoltage(double volts) {
      return new InstantCommand(() -> Robot.intake.setVoltage(volts));
    }

    public static Command setVoltage(SuperstructureConfig config) {
      return setVoltage(config.getIntakeVoltage());
    }
  }
}
