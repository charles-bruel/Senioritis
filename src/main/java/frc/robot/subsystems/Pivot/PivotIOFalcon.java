package frc.robot.subsystems.Pivot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class PivotIOFalcon implements PivotIO {
  private final WPI_TalonFX pivotMotorRight;
  private final WPI_TalonFX pivotMotorLeft;

  private final DutyCycleEncoder absoluteEncoder;

  public PivotIOFalcon() {
    pivotMotorRight = new WPI_TalonFX(Constants.RobotMap.PIVOT_RIGHT);
    pivotMotorLeft = new WPI_TalonFX(Constants.RobotMap.PIVOT_LEFT);

    pivotMotorRight.setInverted(true);

    pivotMotorLeft.follow(pivotMotorRight);
    pivotMotorLeft.setInverted(false);

    pivotMotorRight.setNeutralMode(NeutralMode.Coast);
    pivotMotorLeft.setNeutralMode(NeutralMode.Coast);

    pivotMotorRight.configSupplyCurrentLimit(Constants.PivotConstants.CURRENT_LIMIT);
    pivotMotorLeft.configSupplyCurrentLimit(Constants.PivotConstants.CURRENT_LIMIT);

    absoluteEncoder = new DutyCycleEncoder(Constants.RobotMap.PIVOT_ENCODER);
    absoluteEncoder.setPositionOffset(Constants.PivotConstants.ENCODER_OFFSET);
  }

  @Override
  public void updateInputs(PivotInputs inputs) {
    inputs.absoluteEncoderAngle = absoluteEncoder.getAbsolutePosition() * -360 + 109;
    inputs.motorEncoderAngle =
        pivotMotorRight.getSelectedSensorPosition(0)
            * (360.0 / 2048.0)
            * (1 / Constants.PivotConstants.GEAR_RATIO);

    inputs.motorRightOutputAmpsSupply = pivotMotorRight.getSupplyCurrent();
    inputs.motorLeftOutputAmpsSupply = pivotMotorLeft.getSupplyCurrent();

    inputs.motorRightOutputAmpsStator = pivotMotorRight.getStatorCurrent();
    inputs.motorLeftOutputAmpsStator = pivotMotorLeft.getStatorCurrent();

    inputs.motorRightOutputVolts = pivotMotorRight.getMotorOutputVoltage();
    inputs.motorLeftOutputVolts = pivotMotorLeft.getMotorOutputVoltage();
  }

  @Override
  public void setVoltage(double volts) {
    pivotMotorRight.setVoltage(volts);
  }
}
