package frc.robot.commands.shooter.hood;

import static frc.robot.RobotContainer.hood;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class HomeHood extends Command {
  private final double GOAL_THRESHOLD = 1 / 360.0;

  public HomeHood() {
    addRequirements(hood);
  }

  @Override
  public void initialize() {
    hood.createNewConfig();
    System.out.println("THRES: " + GOAL_THRESHOLD);
  }

  @Override
  public void execute() {
    hood.moveToPos(Constants.Shooter.Turret.TURRET_HOME_POS);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(Constants.Shooter.Turret.TURRET_HOME_POS - hood.getMotorPosition()) < GOAL_THRESHOLD;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("GOAL: " + Constants.Shooter.Turret.TURRET_HOME_POS + "; END: " + hood.getMotorPosition() + "; DIFF" + Math.abs(Constants.Shooter.Turret.TURRET_HOME_POS - hood.getMotorPosition()));
    hood.stopMotor();
  }
}