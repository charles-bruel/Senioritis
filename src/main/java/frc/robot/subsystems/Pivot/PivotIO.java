package frc.robot.subsystems.Pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public class PivotInputs {
        public double motorEncoderAngle;
        public double absoluteEncoderAngle;

        public double motor1OutputVolts;
        public double motor1OutputAmpsSupply;
        public double motor1OutputAmpsStator;
        public double motor2OutputVolts;
        public double motor2OutputAmpsSupply;
        public double motor2OutputAmpsStator;

    }

    public void updateInputs(PivotInputs inputs);
    
    public void setVoltage(double volts);
}
