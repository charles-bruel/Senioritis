package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class ArmIOSparkMAX implements ArmIO {

  private final CANSparkMax armMotor;
  private final DutyCycleEncoder absoluteEncoder;

  public ArmIOSparkMAX() {
    armMotor = new CANSparkMax(Constants.RobotMap.ARM, MotorType.kBrushless);
    absoluteEncoder = new DutyCycleEncoder(Constants.RobotMap.ARM_ENCODER);

    armMotor.setCANTimeout(Constants.CAN_TIMEOUT_MS);
    armMotor.setSmartCurrentLimit(Constants.ArmConstants.CURRENT_LIMIT);
    armMotor.setInverted(false);
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.getEncoder().setPositionConversionFactor(1 / Constants.ArmConstants.GEAR_RATIO);
    absoluteEncoder.setPositionOffset(Constants.ArmConstants.ENCODER_OFFSET);
    absoluteEncoder.setDistancePerRotation(Constants.ArmConstants.DIST_PER_ROTATION);
  }

  @Override
  public void updateInputs(ArmInputs inputs) {
    inputs.absoluteEncoderHeight = absoluteEncoder.getDistance();
    inputs.motorEncoderHeight = armMotor.getEncoder().getPosition();
    inputs.motorOutputAmps = armMotor.getOutputCurrent();
    inputs.motorOutputVolts = armMotor.getAppliedOutput();
  }

  @Override
  public void setVoltage(double volts) {
    armMotor.setVoltage(volts);
  }
}
