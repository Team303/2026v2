package frc.robot.commands.TurretCommands;


import static frc.robot.RobotContainer.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class TurnToHub extends Command {
  private final double GOAL_THRESHOLD = 0.0 / 360.0;
  private double goal; //Rotations
  Turret turret;

  public TurnToHub(Turret turret) {
    addRequirements(turret);
    this.turret = turret;
  }

  @Override
  public void initialize() {
    //turret.createNewConfig();
    this.goal = turret.getTurretTurnPos() / 360.0;
    System.out.println("GOAL: " + goal);
  }

  @Override
  public void execute() {
    System.out.println("GOALLLLLL: " + turret.getTurretTurnPos());
    goal = turret.getTurretTurnPos() / 360.0;

   // System.out.println("Rot goal: " + goal + " | Angle Goal: " + -turret.getTurretTurnPos());
    turret.moveToPos(goal);
    System.out.println("GOAL: " + goal + "; END: " + turret.getMotorPosition() + "; DIFF" + Math.abs(goal - turret.getMotorPosition()));
  }

  @Override
  public boolean isFinished() {
    return Math.abs(turret.getMotorPosition()) > Constants.Shooter.Turret.HARD_MAX_TURRET_ROTATION;
    //return false;//Math.abs(goal - turret.getMotorPosition()) < GOAL_THRESHOLD;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("GOAL: " + goal + "; END: " + turret.getMotorPosition() + "; DIFF" + Math.abs(goal - turret.getMotorPosition()));
    turret.stopMotor();
  }
}