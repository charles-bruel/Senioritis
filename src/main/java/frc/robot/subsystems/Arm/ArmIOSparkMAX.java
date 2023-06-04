package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmIOSparkMAX implements ArmIO {

  private final CANSparkMax armMotor;
  private final DutyCycleEncoder absoluteEncoder;

  public ArmIOSparkMAX() {
    armMotor = new CANSparkMax(RobotMap.ARM, MotorType.kBrushless);
    absoluteEncoder = new DutyCycleEncoder(RobotMap.ARM_ENCODER);

    armMotor.setCANTimeout(Constants.CAN_TIMEOUT_MS);
    armMotor.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT);
    armMotor.setInverted(false);
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.getEncoder().setPositionConversionFactor(1 / ArmConstants.GEAR_RATIO);

    absoluteEncoder.setPositionOffset(ArmConstants.ENCODER_OFFSET);
    absoluteEncoder.setDistancePerRotation(ArmConstants.DIST_PER_ROTATION);
  }

  @Override
  public void updateInputs(ArmInputs inputs) {
    inputs.absoluteEncoderAngle = absoluteEncoder.getAbsolutePosition() * 360;
    inputs.absoluteEncoderHeight = absoluteEncoder.getDistance();
    inputs.motorEncoderHeight =
        armMotor.getEncoder().getPosition() * ArmConstants.DIST_PER_ROTATION;
    inputs.motorEncoderAngle = armMotor.getEncoder().getPosition();
    inputs.motorOutputAmps = armMotor.getOutputCurrent();
    inputs.motorOutputVolts = armMotor.getAppliedOutput();
  }

  @Override
  public void setVoltage(double volts) {
    armMotor.setVoltage(volts);
  }
}
