package frc.robot.subsystems.Chassis.Modules;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.utilities.ModuleInfo;

public class ModuleIOSparkMAX implements ModuleIO {

  CANSparkMax driver;
  CANSparkMax azimuth;
  WPI_CANCoder CANCoder;
  CANCoderConfiguration CANCoderConfig = new CANCoderConfiguration();
  private final ModuleInfo information;

  private RelativeEncoder getDriveEncoder() {
    return driver.getEncoder();
  }

  private RelativeEncoder getAziEncoder() {
    return azimuth.getEncoder();
  }

  public void applyVoltageForCharacterization(double voltage) {
    driver.setVoltage(voltage);
  }

  public WPI_CANCoder getCANCoder() {
    return CANCoder;
  }

  public ModuleIOSparkMAX(ModuleInfo information) {
    this.information = information;
    CANCoder = new WPI_CANCoder(information.getCANCoder());
    CANCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    CANCoderConfig.magnetOffsetDegrees = information.getOffset();
    CANCoder.configAllSettings(CANCoderConfig);
    CANCoder.configMagnetOffset(information.getOffset());
    driver = new CANSparkMax(this.information.getDriveCANId(), MotorType.kBrushless);
    azimuth = new CANSparkMax(this.information.getAziCANId(), MotorType.kBrushless);

    // driver.restoreFactoryDefaults();
    // azimuth.restoreFactoryDefaults();

    driver.setCANTimeout(Constants.CAN_TIMEOUT_MS);
    azimuth.setCANTimeout(Constants.CAN_TIMEOUT_MS);

    driver.setSmartCurrentLimit(Constants.DriveConstants.DRIVE_CURRENT_LIMIT);
    azimuth.setSmartCurrentLimit(Constants.DriveConstants.AZI_CURRENT_LIMIT);

    for (int i = 0; i < 30; i++) {
      azimuth.setInverted(true);
      driver.setInverted(true);
    }

    driver.setIdleMode(IdleMode.kBrake);
    azimuth.setIdleMode(IdleMode.kBrake);

    getDriveEncoder().setPositionConversionFactor(Constants.DriveConstants.DIST_PER_PULSE);

    getDriveEncoder().setVelocityConversionFactor((Constants.DriveConstants.DIST_PER_PULSE / 60));

    getAziEncoder().setPositionConversionFactor(7.0 / 150.0 * 360.0);
    getAziEncoder().setVelocityConversionFactor(7.0 / 150.0 * 360.0);

    seed();
  }

  public void seed() {
    getAziEncoder().setPosition(getCANCoder().getAbsolutePosition());
  }

  @Override
  public void updateInputs(ModuleInputs inputs) {

    inputs.aziAbsoluteEncoderDegrees = CANCoder.getAbsolutePosition();

    inputs.aziOutputVolts = azimuth.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.aziTempCelcius = azimuth.getMotorTemperature();
    inputs.aziCurrentDrawAmps = azimuth.getOutputCurrent();
    inputs.aziEncoderPositionDeg = getAziEncoder().getPosition() - 180;
    inputs.aziEncoderVelocityDegPerSecond = getAziEncoder().getVelocity();

    inputs.driveEncoderPositionMetres = getDriveEncoder().getPosition();
    inputs.driveEncoderVelocityMetresPerSecond = getDriveEncoder().getVelocity();
    inputs.driveOutputVolts = driver.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.driveCurrentDrawAmps = driver.getOutputCurrent();
    inputs.driveTempCelcius = driver.getMotorTemperature();
  }

  @Override
  public void setAzimuthVoltage(double aziVolts) {
    azimuth.setVoltage(aziVolts);
  }

  @Override
  public void setDriveVoltage(double driveVolts) {
    driver.setVoltage(driveVolts);
  }
}
