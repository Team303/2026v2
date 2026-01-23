package frc.robot.commands.shooter.turret;

import static frc.robot.RobotContainer.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Shooter;

public class HomeTurret extends Command {
  private final double GOAL_THRESHOLD = 10.0;

  public HomeTurret() {
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    turret.createNewConfig();
  }

  @Override
  public void execute() {
    turret.moveToPos(Shooter.TURRET_HOME_POS);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(Shooter.TURRET_HOME_POS - turret.getAbsolutePosition()) < GOAL_THRESHOLD;
  }

  @Override
  public void end(boolean interrupted) {
    turret.stopMotor();
  }
}
