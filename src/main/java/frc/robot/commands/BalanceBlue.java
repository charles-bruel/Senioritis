package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.utilities.MotionHandler.MotionMode;
import org.littletonrobotics.junction.Logger;

public class BalanceBlue extends SequentialCommandGroup {
  class BangBang {
    double setpoint, tolerance, lastMeasurement;
    SlewRateLimiter limiter;
    public double speed;
    private double prevError = 0;

    private double decayRate = 0.65;

    public BangBang(double setpoint, double tolerance) {
      this.init();
      this.setpoint = setpoint;
      this.tolerance = tolerance;
      this.limiter = new SlewRateLimiter(this.speed);
    }

    public void init() {
      this.speed = 0.8;
      this.lastMeasurement = 0;
      this.prevError = 0;
    }

    double calculate(double measurement) {
      double currentError = measurement - setpoint;
      double rollSpeed = Math.abs(measurement - lastMeasurement);
      if (Math.signum(prevError) != Math.signum(currentError)) {
        speed *= decayRate;
      }
      lastMeasurement = measurement;
      prevError = currentError;

      double out = limiter.calculate(speed);
      Logger.getInstance().recordOutput("PIDBridge/speed", out);
      Logger.getInstance().recordOutput("PIDBridge/currentError", currentError);
      Logger.getInstance().recordOutput("PIDBridge/rollspeed", rollSpeed);
      if (currentError > tolerance && rollSpeed < 0.25) {
        return out;
      } else if (currentError < -tolerance && rollSpeed < 0.25) {
        return -out;
      }
      return 0;
    }
  }

  private double rampSpeed = 1.75;

  public BalanceBlue() {
    BangBang controller = new BangBang(0, 4.5);
    addCommands(
        new InstantCommand(() -> Robot.motionMode = MotionMode.NULL),
        new RunCommand(
                () ->
                    Robot.swerveDrive.setModuleStates(
                        DriveConstants.KINEMATICS.toSwerveModuleStates(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                rampSpeed, 0, 0, Robot.swerveDrive.getYaw()))))
            .until(() -> Math.abs(Robot.swerveDrive.filteredRollVal) > 12),
        new RunCommand(
            () ->
                Robot.swerveDrive.setModuleStates(
                    DriveConstants.KINEMATICS.toSwerveModuleStates(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            controller.calculate(Robot.swerveDrive.filteredRollVal),
                            0,
                            0,
                            Robot.swerveDrive.getYaw())))));
  }
}
