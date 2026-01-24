package frc.robot.commands.shooter.turret;

import static frc.robot.RobotContainer.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ZeroTurret extends Command {
  private final double GOAL_THRESHOLD = 1 / 360.0;

  public ZeroTurret() {
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    turret.createNewConfig();
    System.out.println("THRES: " + GOAL_THRESHOLD);
  }

  @Override
  public void execute() {
    turret.moveToPos(0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(0 - turret.getMotorPosition()) < GOAL_THRESHOLD;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("GOAL: " + 0 + "; END: " + turret.getMotorPosition() + "; DIFF" + Math.abs(0 - turret.getMotorPosition()));
    turret.stopMotor();
  }
}