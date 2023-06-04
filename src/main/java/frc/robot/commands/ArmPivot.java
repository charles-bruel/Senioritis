package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.utilities.SuperstructureConfig;

public class ArmPivot extends SequentialCommandGroup {

  public ArmPivot(SuperstructureConfig target) {
    addCommands(
        ArmSubsystem.Commands.setHeightBlocking(ArmConstants.RETRACTED_POSITION),
        PivotSubsystem.Commands.setPositionBlocking(target),
        ArmSubsystem.Commands.setHeightBlocking(target));
  }
}
