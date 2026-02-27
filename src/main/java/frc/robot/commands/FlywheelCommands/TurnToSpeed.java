package frc.robot.commands.FlywheelCommands;

import static frc.robot.RobotContainer.flywheel;

import com.pathplanner.lib.path.GoalEndState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class TurnToSpeed extends Command {

  private double goal_speed;

  public TurnToSpeed(double GOAL_SPEED) {
    this.goal_speed = GOAL_SPEED;
    addRequirements(flywheel);
  }

  @Override
  public void initialize() {
    //flywheel.createNewConfig();
  }

  @Override
  public void execute() {
     flywheel.getToSpeed(goal_speed); //2520 - 6
    //flywheel.rightFlywheelMotor.setVoltage(6);

    // flywheel.leftFlywheelMotor.set(1);
    // flywheel.rightFlywheelMotor.set(-1);
    
    System.out.println("left v: " + flywheel.getLeftMotorSpeed());
    System.out.println("right v: " + flywheel.getRightMotorSpeed());


    System.out.println("left a: " + flywheel.getLeftMotorAccel());
    System.out.println("right a: " + flywheel.getRightMotorAccel());

    //hood.moveToPos(Constants.Shooter.Turret.TURRET_HOME_POS);

    // flywheel.leftFlywheelMotor.set(0.5);
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