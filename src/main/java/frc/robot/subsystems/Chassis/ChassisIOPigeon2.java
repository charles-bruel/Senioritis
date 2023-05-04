package frc.robot.subsystems.Chassis;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.RobotMap;
import org.littletonrobotics.junction.Logger;

public class ChassisIOPigeon2 implements ChassisIO {

  private final Pigeon2 gyro;

  public ChassisIOPigeon2() {
    gyro = new Pigeon2(RobotMap.GYRO_PORT);
    gyro.zeroGyroBiasNow();
    gyro.setYaw(0);
  }

  @Override
  public void updateInputs(ChassisInputs inputs) {
    // inputs.gyroCompassHeading = gyro.getAbsoluteCompassHeading();
    inputs.previousgyroPitchPosition = inputs.gyroPitchPosition;
    inputs.gyroPitchPosition = gyro.getPitch();
    inputs.gyroRollPosition = gyro.getRoll();
    inputs.gyroYawPosition = gyro.getYaw(); // gyro faces forwards on the robot
  }

  @Override
  public void resetGyro(Rotation2d rotation2d) {
    Logger.getInstance().recordOutput("Reset gyro to", rotation2d.getDegrees());
    gyro.setYaw(rotation2d.getDegrees());
  }

  public void zeroGyro() {
    Logger.getInstance().recordOutput("Reset gyro to", 0);
    gyro.zeroGyroBiasNow();
    gyro.setYaw(0);
  }
}
