package frc.robot.subsystems.Pivot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;

// TODO: Work out encoders
public class PivotIOFalcon implements PivotIO {
    private final WPI_TalonFX pivotMotor1;
    private final WPI_TalonFX pivotMotor2;

    public PivotIOFalcon() {
        pivotMotor1 = new WPI_TalonFX(Constants.RobotMap.PIVOT_1);
        pivotMotor2 = new WPI_TalonFX(Constants.RobotMap.PIVOT_2);

        pivotMotor2.follow(pivotMotor1);
        pivotMotor2.setInverted(true);

        pivotMotor1.setNeutralMode(NeutralMode.Brake);
        pivotMotor2.setNeutralMode(NeutralMode.Brake);

        //TODO: Current limit the falcons
    }

    @Override
    public void updateInputs(PivotInputs inputs) {

        // TODO: Determine if it should be stator current or supply current
        inputs.motor1OutputAmps = pivotMotor1.getStatorCurrent();
        inputs.motor2OutputAmps = pivotMotor2.getStatorCurrent();

        inputs.motor1OutputVolts = pivotMotor1.getMotorOutputVoltage();
        inputs.motor2OutputVolts = pivotMotor2.getMotorOutputVoltage();
    }

    @Override
    public void setVoltage(double volts) {
        pivotMotor1.setVoltage(volts);  
    }
}
