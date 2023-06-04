package frc.robot.utilities;

import edu.wpi.first.math.controller.PIDController;

public class PIDFFController extends PIDController {
  private AbstractFeedForward feedforward;

  public PIDFFController(PIDFFGains gains) {
    super(gains.kP.get(), gains.kI.get(), gains.kD.get());

    feedforward = gains.createAbstractFeedForward();
    setTolerance(gains.tolerance.get());
  }

  @Override
  public double calculate(double measurement, double setpoint) {
    setSetpoint(setpoint);
    return calculate(measurement);
  }

  @Override
  public double calculate(double measurement) {
    return super.calculate(measurement) + feedforward.calculate(measurement, getVelocityError());
  }
}
