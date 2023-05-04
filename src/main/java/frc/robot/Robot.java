// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Chassis.ChassisIOPigeon2;
import frc.robot.subsystems.Chassis.ChassisSubsystem;
import frc.robot.subsystems.Chassis.Modules.ModuleIOSparkMAX;
import frc.robot.utilities.MotionHandler.MotionMode;

public class Robot extends LoggedRobot {
public static ChassisSubsystem swerveDrive;


  public static MotionMode motionMode = MotionMode.LOCKDOWN;

  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1);

  private Command m_autonomousCommand;

  @Override
  public void robotInit() {
    swerveDrive = new ChassisSubsystem(new ChassisIOPigeon2(), new ModuleIOSparkMAX(Constants.DriveConstants.FRONT_LEFT), new ModuleIOSparkMAX(Constants.DriveConstants.FRONT_RIGHT), new ModuleIOSparkMAX(Constants.DriveConstants.BACK_LEFT), new ModuleIOSparkMAX(Constants.DriveConstants.BACK_RIGHT));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = null;

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
