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
    armMotor.setIdleMode(IdleMode.kCoast);
    armMotor.getEncoder().setPositionConversionFactor(1 / ArmConstants.GEAR_RATIO);
  }

  @Override
  public void updateInputs(ArmInputs inputs) {
    // This gets the base (multiple of 360) of the cumulative absolute encoder angle. This allows us
    // to add
    // back the encoder angle and get the next cumulative value.
    // Desync should not happen because there is only one call to the absolute encoder for position.
    double absoluteEncoderCumAngleBase =
        inputs.absoluteEncoderCumAngle - inputs.absoluteEncoderAngle;
    double previousAbsoluteEncoderAngle = inputs.absoluteEncoderAngle;

    // This gets the encoder position normalized to 0 - 360, and where the encoder is at 0 at the 0
    // position
    inputs.absoluteEncoderAngle =
        ((absoluteEncoder.getAbsolutePosition() * 360) - Constants.ArmConstants.ENCODER_OFFSET)
            % 360;

    // We now determine if it has rolled over or under
    // We determine the change in angle assuming all 3 scenarios (regular, roll over, roll under)
    // and check which is smallest
    double deltaAssumeNothing =
        Math.abs(previousAbsoluteEncoderAngle - inputs.absoluteEncoderAngle);
    double deltaAssumeRollOver =
        Math.abs(previousAbsoluteEncoderAngle - (inputs.absoluteEncoderAngle + 360));
    double deltaAssumeRollUnder =
        Math.abs((previousAbsoluteEncoderAngle + 360) - inputs.absoluteEncoderAngle);

    if (deltaAssumeNothing < deltaAssumeRollUnder && deltaAssumeNothing < deltaAssumeRollOver) {
      // All is good, do nothing
    } else if (deltaAssumeRollUnder < deltaAssumeNothing
        && deltaAssumeRollUnder < deltaAssumeRollOver) {
      // The encoder rolled under, so we subtract a revolution
      absoluteEncoderCumAngleBase -= 360;
    } else {
      // The encoder rolled over, so we add a revolution
      absoluteEncoderCumAngleBase += 360;
    }
    inputs.absoluteEncoderCumAngle = absoluteEncoderCumAngleBase + inputs.absoluteEncoderAngle;

    inputs.absoluteEncoderHeight =
        inputs.absoluteEncoderCumAngle * Constants.ArmConstants.DIST_PER_ROTATION;

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
