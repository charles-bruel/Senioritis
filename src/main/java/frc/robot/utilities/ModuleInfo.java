package frc.robot.utilities;

import edu.wpi.first.math.geometry.Translation2d;
import lombok.Builder;
import lombok.Getter;

@Builder
public class ModuleInfo {
  public enum SwerveModuleName {
    FRONT_LEFT("FrontLeft"),
    FRONT_RIGHT("FrontRight"),
    BACK_LEFT("BackLeft"),
    BACK_RIGHT("BackRight");

    private final String string;

    SwerveModuleName(String name) {
      string = name;
    }

    @Override
    public String toString() {
      return string;
    }
  }

  @Getter private SwerveModuleName name;
  @Getter private PIDFFGains driveGains;
  @Getter private PIDFFGains azimuthGains;
  @Getter private int driveCANId;
  @Getter private int aziCANId;
  @Getter private int CANCoder;
  @Getter private double offset;
  @Getter private Translation2d location;
}
