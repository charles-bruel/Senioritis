package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import frc.robot.Constants;

public class ArmIOSparkMAX implements ArmIO {

  private final CANSparkMax armMotor;
  private final SparkMaxAbsoluteEncoder absoluteEncoder;

  public ArmIOSparkMAX() {
    armMotor = new CANSparkMax(Constants.RobotMap.ARM, MotorType.kBrushless);
    absoluteEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);

    armMotor.setCANTimeout(Constants.CAN_TIMEOUT_MS);
    armMotor.setSmartCurrentLimit(Constants.ArmConstants.CURRENT_LIMIT);
    armMotor.setInverted(false);
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.getEncoder().setPositionConversionFactor(Constants.ArmConstants.GEAR_RATIO);
  }

  @Override
  public void updateInputs(ArmInputs inputs) {
    inputs.absoluteEncoderHeight = absoluteEncoder.getPosition();
    inputs.motorEncoderHeight = armMotor.getEncoder().getPosition();
    inputs.motorOutputAmps = armMotor.getOutputCurrent();
    inputs.motorOutputVolts = armMotor.getAppliedOutput();
  }

  @Override
  public void setVoltage(double volts) {
    armMotor.setVoltage(volts);
  }
}
