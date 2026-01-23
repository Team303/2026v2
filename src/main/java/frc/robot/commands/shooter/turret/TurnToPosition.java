package frc.robot.commands.shooter.turret;

import static frc.robot.RobotContainer.turret;
import static frc.robot.subsystems.turret.Turret.HOME_POS;

import edu.wpi.first.wpilibj2.command.Command;

public class TurnToPosition extends Command {
  private final double GOAL_THRESHOLD = 0 / 360.0;
  private double goal; //Rotations

  public TurnToPosition(double goal) {
    addRequirements(turret);
    this.goal = HOME_POS.getAsDouble();
  }

  @Override
  public void initialize() {
    turret.createNewConfig();
    this.goal = HOME_POS.getAsDouble();
    System.out.println(goal);
  }

  @Override
  public void execute() {
    turret.moveToPos(goal);
  }

  @Override
  public boolean isFinished() {
    //System.out.println("isFinished Diff: " + Math.abs(goal - turret.throughBore.getPosition().getValueAsDouble()) + "; Thres: " + GOAL_THRESHOLD);
    return Math.abs(goal - turret.getRelativePosition()) < GOAL_THRESHOLD;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("GOAL: " + turret.getRelativePosition());
    turret.stopMotor();
  }
}
