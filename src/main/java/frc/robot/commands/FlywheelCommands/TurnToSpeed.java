package frc.robot.commands.FlywheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import static frc.robot.subsystems.Flywheel.GOAL_SPEED;
import static frc.robot.RobotContainer.turret;

public class TurnToSpeed extends Command {

  private double goal_speed;
  private Flywheel flywheel;

  public TurnToSpeed(Flywheel flywheel) {
    this.goal_speed = 0;
    this.flywheel = flywheel;
    addRequirements(flywheel);
  }

  @Override
  public void initialize() {
    goal_speed = flywheel.getShootOnMoveSpeed(
        turret.getVirtualTargetDistance(),
        turret.getVelocityComponentTowardHub());
    goal_speed -= 2;
  }

  @Override
  public void execute() {
    goal_speed = flywheel.getShootOnMoveSpeed(
        turret.getVirtualTargetDistance(),
        turret.getVelocityComponentTowardHub());
    System.out.println("goal_speed: " + goal_speed);
    flywheel.getToSpeed(goal_speed);
    //flywheel.rightFlywheelMotor.setVoltage(6);

    // flywheel.leftFlywheelMotor.set(1);
    // flywheel.rightFlywheelMotor.set(-1);
    
    System.out.println("left v: " + flywheel.getLeftMotorSpeed());
    System.out.println("right v: " + flywheel.getRightMotorSpeed());


    // System.out.println("left a: " + flywheel.getLeftMotorAccel());
    // System.out.println("right a: " + flywheel.getRightMotorAccel());
    System.out.println("kicker speed: " + flywheel.getKickerMotorSpeed());

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