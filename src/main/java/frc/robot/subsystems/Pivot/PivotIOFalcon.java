package frc.robot.subsystems.Pivot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class PivotIOFalcon implements PivotIO {
    private final WPI_TalonFX pivotMotor1;
    private final WPI_TalonFX pivotMotor2;

    private final DutyCycleEncoder absoluteEncoder;

    public PivotIOFalcon() {
        pivotMotor1 = new WPI_TalonFX(Constants.RobotMap.PIVOT_1);
        pivotMotor2 = new WPI_TalonFX(Constants.RobotMap.PIVOT_2);

        pivotMotor2.follow(pivotMotor1);
        pivotMotor2.setInverted(true);

        pivotMotor1.setNeutralMode(NeutralMode.Brake);
        pivotMotor2.setNeutralMode(NeutralMode.Brake);

        //TODO: Current limit the falcons

        absoluteEncoder = new DutyCycleEncoder(Constants.RobotMap.PIVOT_ENCODER);
        absoluteEncoder.setPositionOffset(Constants.PivotConstants.ENCODER_OFFSET);
    }

    @Override
    public void updateInputs(PivotInputs inputs) {
        inputs.absoluteEncoderAngle = absoluteEncoder.getAbsolutePosition();
        inputs.motorEncoderAngle = 
            pivotMotor1.getSelectedSensorPosition(0)
            * (360.0/2048.0)
            * (1/Constants.PivotConstants.GEAR_RATIO);

        inputs.motor1OutputAmpsSupply = pivotMotor1.getSupplyCurrent();
        inputs.motor2OutputAmpsSupply = pivotMotor2.getSupplyCurrent();

        inputs.motor1OutputAmpsStator = pivotMotor1.getStatorCurrent();
        inputs.motor2OutputAmpsStator = pivotMotor2.getStatorCurrent();

        inputs.motor1OutputVolts = pivotMotor1.getMotorOutputVoltage();
        inputs.motor2OutputVolts = pivotMotor2.getMotorOutputVoltage();
    }

    @Override
    public void setVoltage(double volts) {
        pivotMotor1.setVoltage(volts);  
    }
}
