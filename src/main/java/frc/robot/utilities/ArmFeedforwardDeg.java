package frc.robot.utilities;

import edu.wpi.first.math.controller.ArmFeedforward;

public class ArmFeedforwardDeg extends ArmFeedforward {

  private static final double DEG2RADD = Math.PI / 180.0;

  public ArmFeedforwardDeg(double ks, double kg, double kv) {
    super(ks, kg, kv);
  }

  public ArmFeedforwardDeg(double ks, double kg, double kv, double ka) {
    super(ks, kg, kv, ka);
  }

  @Override
  public double calculate(
      double positionDegrees, double velocityDegreesPerSec, double accelDegreesPerSecSquared) {
    return ks * Math.signum(velocityDegreesPerSec * DEG2RADD)
        + kg * Math.cos(positionDegrees * DEG2RADD)
        + kv * velocityDegreesPerSec * DEG2RADD
        + ka * accelDegreesPerSecSquared * DEG2RADD;
  }
}
