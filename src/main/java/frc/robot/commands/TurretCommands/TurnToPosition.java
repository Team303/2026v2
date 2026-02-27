package frc.robot.commands.TurretCommands;

import static frc.robot.subsystems.Turret.GOAL_POS;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class TurnToPosition extends Command {
  private final double GOAL_THRESHOLD = 0.25 / 360.0;
  private double goal; //Rotations
  Turret turret;

  public TurnToPosition(Turret turret, double goal) {
    addRequirements(turret);
    this.goal = goal;
    this.turret = turret;
  }

  @Override
  public void initialize() {
    //this.goal = GOAL_POS.getAsDouble();
    System.out.println("THRES: " + GOAL_THRESHOLD);
  }

  @Override
  public void execute() {
    turret.moveToPos(goal);
  }

  @Override
  public boolean isFinished() {
    //System.out.println("isFinished Diff: " + Math.abs(goal - turret.throughBore.getPosition().getValueAsDouble()) + "; Thres: " + GOAL_THRESHOLD);
    return Math.abs(goal - turret.getMotorPosition()) < GOAL_THRESHOLD;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("GOAL: " + goal + "; END: " + turret.getMotorPosition() + "; DIFF" + Math.abs(goal - turret.getMotorPosition()));
    turret.stopMotor();
  }
}