package frc.robot.commands.TurretCommands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class HomeTurret extends Command {
  private final double GOAL_THRESHOLD = 0.0 / 360.0;
  Turret turret;

  public HomeTurret(Turret turret) {
    addRequirements(turret);
    this.turret = turret;
  }

  @Override
  public void initialize() {
    //turret.createNewConfig();
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