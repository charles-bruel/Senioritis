package frc.robot.subsystems.Chassis;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ChassisIO {

  @AutoLog
  public static class ChassisInputs {
    public double gyroYawPosition = 10.0;
    public double gyroPitchPosition = 0.0;
    public double previousgyroPitchPosition = 0.0;
    public double gyroRollPosition = 0.0;
    public double gyroCompassHeading = 0.0;
  }

  /**
   * Updates input values with the given SwerveInputs instance.
   *
   * @param inputs SwerveInputs instance
   */
  public void updateInputs(ChassisInputs inputs);

  /**
   * Sets the gyro to the given rotation.
   *
   * @param rotation2d The desired rotation
   */
  public void resetGyro(Rotation2d rotation2d);

  public void zeroGyro();
}
