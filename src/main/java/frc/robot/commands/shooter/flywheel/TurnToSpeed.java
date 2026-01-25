package frc.robot.commands.shooter.flywheel;

import static frc.robot.RobotContainer.flywheel;
import static frc.robot.subsystems.shooter.Flywheel.GOAL_SPEED;

import com.pathplanner.lib.path.GoalEndState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class TurnToSpeed extends Command {

  public TurnToSpeed() {
    addRequirements(flywheel);
  }

  @Override
  public void initialize() {
    flywheel.createNewConfig();
  }

  @Override
  public void execute() {
    flywheel.getToSpeed(GOAL_SPEED.getAsDouble());
    //hood.moveToPos(Constants.Shooter.Turret.TURRET_HOME_POS);
  }

  @Override
  public boolean isFinished() {
    return false;//Math.abs(Constants.Shooter.Turret.TURRET_HOME_POS - hood.getMotorPosition()) < GOAL_THRESHOLD;
  }

  @Override
  public void end(boolean interrupted) {
    //System.out.println("GOAL: " + Constants.Shooter.Turret.TURRET_HOME_POS + "; END: " + hood.getMotorPosition() + "; DIFF" + Math.abs(Constants.Shooter.Turret.TURRET_HOME_POS - hood.getMotorPosition()));
    flywheel.stopMotors();
  }
}