package frc.robot.commands.shooter.turret;

import static frc.robot.RobotContainer.turret;

import edu.wpi.first.wpilibj2.command.Command;

public class TurnToPosition extends Command {
  private final double GOAL_THRESHOLD = 10.0;
  private double goal; // Theta? god knows

  public TurnToPosition(double goal) {
    addRequirements(turret);
    this.goal = goal;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    turret.moveToPos(goal);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(goal - turret.getAbsolutePosition()) < GOAL_THRESHOLD;
  }

  @Override
  public void end(boolean interrupted) {
    turret.stopMotor();
  }
}
