package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
}
