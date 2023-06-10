package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Robot;
import frc.robot.utilities.SuperstructureConfig;

public class SetSuperstructure extends CommandBase {

  private PathType pathType;
  private int pathStage;
  private final SuperstructureConfig target;

  public SetSuperstructure(SuperstructureConfig target) {
    addRequirements(Robot.pivot, Robot.arm);
    this.target = target;
  }

  @Override
  public void initialize() {
    pathType = PathType.FULL;
    pathStage = 0;
  }

  @Override
  public void execute() {
    switch (pathType) {
      case FULL:
        executeFull();
        break;
    }
  }

  private void executeFull() {
    switch (pathStage) {
      case 0:
        Robot.arm.setTargetHeight(ArmConstants.RETRACTED_POSITION);
        if (Robot.arm.isAtTarget()) pathStage = 1;
        break;
      case 1:
        Robot.pivot.setTargetAngle(target.getPivotPosition());
        if (Robot.pivot.isAtTarget()) pathStage = 2;
        break;
      case 2:
        Robot.arm.setTargetHeight(target.getArmHeight());
        if (Robot.arm.isAtTarget()) pathStage = 3;
        break;
    }
  }

  @Override
  public boolean isFinished() {
    switch (pathType) {
      case FULL:
        return pathStage == 3;
    }
    return true;
  }

  public static enum PathType {
    FULL
  }
}
