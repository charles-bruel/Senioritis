package frc.robot.subsystems.Pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public class PivotInputs {
        public double motorEncoderAngle;
        public double absoluteEncoderAngle;

        public double motor1OutputVolts;
        public double motor1OutputAmps;
        public double motor2OutputVolts;
        public double motor2OutputAmps;

    }

    public void updateInputs(PivotInputs inputs);
    
    public void setVoltage(double volts);
}
