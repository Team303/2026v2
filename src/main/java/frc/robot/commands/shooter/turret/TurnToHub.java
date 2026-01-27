package frc.robot.commands.shooter.turret;

import static frc.robot.RobotContainer.turret;

import edu.wpi.first.wpilibj2.command.Command;

public class TurnToHub extends Command {
  private final double GOAL_THRESHOLD = 5 / 360.0;
  private double goal; //Rotations

  public TurnToHub() {
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    turret.createNewConfig();
    this.goal = turret.getTurretTurnPos() / 360.0;
    System.out.println("GOAL: " + goal);
  }

  @Override
  public void execute() {
    goal = -turret.getTurretTurnPos() / 360.0;
    System.out.println("Rot goal: " + goal + " | Angle Goal: " + -turret.getTurretTurnPos());
    turret.moveToPos(goal);
  }

  @Override
  public boolean isFinished() {
    return false;//Math.abs(goal - turret.getMotorPosition()) < GOAL_THRESHOLD;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("GOAL: " + goal + "; END: " + turret.getMotorPosition() + "; DIFF" + Math.abs(goal - turret.getMotorPosition()));
    turret.stopMotor();
  }
}