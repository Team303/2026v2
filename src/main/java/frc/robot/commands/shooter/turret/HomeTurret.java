package frc.robot.commands.shooter.turret;

import static frc.robot.RobotContainer.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class HomeTurret extends Command {
  private final double GOAL_THRESHOLD = 1 / 360.0;

  public HomeTurret() {
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    turret.createNewConfig();
    System.out.println("THRES: " + GOAL_THRESHOLD);
  }

  @Override
  public void execute() {
    turret.moveToPos(Constants.Shooter.Turret.TURRET_HOME_POS);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(Constants.Shooter.Turret.TURRET_HOME_POS - turret.getMotorPosition()) < GOAL_THRESHOLD;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("GOAL: " + Constants.Shooter.Turret.TURRET_HOME_POS + "; END: " + turret.getMotorPosition() + "; DIFF" + Math.abs(Constants.Shooter.Turret.TURRET_HOME_POS - turret.getMotorPosition()));
    turret.stopMotor();
  }
}