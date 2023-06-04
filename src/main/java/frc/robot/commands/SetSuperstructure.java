package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.utilities.SuperstructureConfig;

public class SetSuperstructure extends SequentialCommandGroup {

  public SetSuperstructure(SuperstructureConfig target) {
    addCommands(
        ArmSubsystem.Commands.setHeightAndWait(ArmConstants.RETRACTED_POSITION),
        PivotSubsystem.Commands.setPositionAndWait(target),
        ArmSubsystem.Commands.setHeightAndWait(target));
  }
}
