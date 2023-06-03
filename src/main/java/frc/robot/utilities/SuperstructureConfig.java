package frc.robot.utilities;

import lombok.Builder;
import lombok.Getter;

@Builder
public class SuperstructureConfig {
  @Getter private double pivotPosition;
  @Getter private double armHeight;
  @Getter private double intakeVoltage;
}
